#include "custom_share_manager.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <thread>
#include <filesystem>
#include <fstream>
#include "util.h"
#ifdef _WIN32
#include <windows.h>
#endif

void GazeStatus::set(const hmd2_gaze_status_t *pGazeStatus) { std::memcpy(&data, pGazeStatus, sizeof(data)); }

void GazeStatus::get(hmd2_gaze_status_t *pGazeStatus) const { std::memcpy(pGazeStatus, &data, sizeof(data)); }

void GazeImage::pushToCircularBuffer(const unsigned char *pGazeImage, uint32_t size) {
  const uint32_t copySize = (size == 0 || size > k_gazeImageSlotSize) ? k_gazeImageSlotSize : size;
  const int index = counter % k_gazeImageSlots;

  std::memcpy(&images[k_gazeImageSlotSize * index], pGazeImage, copySize);
  sizes[index] = copySize;
  counter++;
}

int GazeImage::getFromCircularBuffer(unsigned char **gazeImageBuffer) {
  if (counter == 0) {
    *gazeImageBuffer = nullptr;
    return -1;
  }
  int index = (counter - 1) % k_gazeImageSlots;
  *gazeImageBuffer = &images[k_gazeImageSlotSize * index];
  return index;
}

bool GazeImage::copyLatest(unsigned char *pDest, uint32_t destSize, uint32_t *pOutSize) const {
  if (counter == 0) {
    if (pOutSize) {
      *pOutSize = 0;
    }
    return false;
  }

  const int index = (counter - 1) % k_gazeImageSlots;
  const uint32_t available = (sizes[index] == 0 || sizes[index] > k_gazeImageSlotSize) ? k_gazeImageSlotSize : sizes[index];
  const uint32_t copySize = available < destSize ? available : destSize;

  std::memcpy(pDest, &images[k_gazeImageSlotSize * index], copySize);
  if (pOutSize) {
    *pOutSize = copySize;
  }

  // Report a short read rather than pretending the caller got a whole frame.
  return copySize == available;
}

DriverCommand *CommandBuffer::push(const DriverCommand &command) {
  int next_head = (head + 1) % 256;
  if (next_head == tail)
    return nullptr;
  DriverCommand *ptr = &commands[head];
  *ptr = command;
  // Release: the payload copied above must be visible before a consumer can observe this slot.
  std::atomic_ref<bool>(ptr->isFulfilled).store(false, std::memory_order_release);
  head = next_head;
  return ptr;
}

DriverCommand *CommandBuffer::pop() {
  if (head == tail)
    return nullptr;
  DriverCommand *ptr = &commands[tail];
  tail = (tail + 1) % 256;
  return ptr;
}

CustomShareManager *CustomShareManager::m_pInstance = nullptr;
bool CustomShareManager::m_initialized = false;
std::mutex CustomShareManager::m_instanceMutex;

void CustomShareManager::createSingleton() {
  std::lock_guard<std::mutex> lock(m_instanceMutex);
  m_initialized = true;

  CustomShareManager *pInstance = m_pInstance;
  if (!m_pInstance) {
    pInstance = new CustomShareManager;

    pInstance->initialize();
    m_pInstance = pInstance;
  }
}

CustomShareManager *CustomShareManager::getSingleton() {
  std::lock_guard<std::mutex> lock(m_instanceMutex);

  CustomShareManager *pInstance = m_pInstance;
  if (!m_pInstance) {
    pInstance = new CustomShareManager;
    m_pInstance = pInstance;
  }

  return pInstance;
}

void CustomShareManager::initialize() {
  m_gazeStatusBroadcast = CreateIpcBroadcast(CUSTOM_SHARE_NAME("CUSTOM_SHARE_VRT2_WIN_GAZE_STATUS_BCAST"));
  m_gazeStatusMutex = CreateIpcMutex(CUSTOM_SHARE_NAME("CUSTOM_SHARE_VRT2_WIN_GAZE_STATUS_MTX"));

  m_gazeImageBroadcast = CreateIpcBroadcast(CUSTOM_SHARE_NAME("CUSTOM_SHARE_VRT2_WIN_GAZE_IMAGE_BCAST"));
  m_gazeImageMutex = CreateIpcMutex(CUSTOM_SHARE_NAME("CUSTOM_SHARE_VRT2_WIN_GAZE_IMAGE_MTX"));

  for (int i = 0; i < k_maxSlots; i++) {
    char name[128];
    snprintf(name, sizeof(name), CUSTOM_SHARE_NAME("CUSTOM_SHARE_VRT2_WIN_SLOT_OWNER_MTX") "_%d", i);
    m_slotOwnerMutex[i] = CreateIpcMutex(name);
  }

  m_pcmBroadcast = CreateIpcBroadcast(CUSTOM_SHARE_NAME("CUSTOM_SHARE_VRT2_WIN_PCM_BCAST"));
  m_commandBroadcast = CreateIpcBroadcast(CUSTOM_SHARE_NAME("CUSTOM_SHARE_VRT2_WIN_CMD_BCAST"));

  m_commandMutex = CreateIpcMutex(CUSTOM_SHARE_NAME("CUSTOM_SHARE_VRT2_WIN_COMMAND_MTX"));

  m_sharedMemory = CreateIpcSharedMemory(CUSTOM_SHARE_NAME("CUSTOM_SHARE_VRT2_WIN"), sizeof(BufferData));
  m_pBufferData = static_cast<BufferData *>(IpcSharedMemory_Map(m_sharedMemory));

  m_driverActiveMutex = CreateIpcMutex(CUSTOM_SHARE_NAME("CUSTOM_SHARE_VRT2_DRIVER_ACTIVE_MTX"));
  m_driverActiveGuardMutex = CreateIpcMutex(CUSTOM_SHARE_NAME("CUSTOM_SHARE_VRT2_DRIVER_ACTIVE_GUARD_MTX"));
}

#ifdef _WIN32
void CustomShareManager::setupCAPIPath() {
  try {
    std::filesystem::path temp_folder = GetSystemTempFolder();
    std::filesystem::path path_file = temp_folder / "psvr2tk_capi_path.txt";

    HMODULE hModule = NULL;
    GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, (LPCSTR)&CustomShareManager::createSingleton,
                       &hModule);
    if (hModule != NULL) {
      char path[MAX_PATH];
      if (GetModuleFileNameA(hModule, path, MAX_PATH) > 0) {
        std::filesystem::path dllPath(path);
        std::filesystem::path capiPath = dllPath.parent_path();

        std::ofstream outFile(path_file);
        if (outFile.is_open()) {
          if (IsRunningInWine()) {
            outFile << WineGetUnixFileName(capiPath.string());
          } else {
            outFile << capiPath.string();
          }
        }
      }
    }
  } catch (...) {
  }
}
#endif

bool CustomShareManager::getDriverActive() {
  IpcMutex_Lock(m_driverActiveGuardMutex);

  bool active = true;
  if (IpcMutex_TryLock(m_driverActiveMutex)) {
    // If we can lock it, it means the driver doesn't hold the lock, so it's not active.
    IpcMutex_Unlock(m_driverActiveMutex);
    active = false;
  }

  IpcMutex_Unlock(m_driverActiveGuardMutex);

  return active;
}

// Returns true once this process owns the driver-active mutex, false if it could not be taken within the timeout (which means another driver instance is holding it
bool CustomShareManager::claimDriverMutex() {
  IpcMutex_Lock(m_driverActiveGuardMutex);

  // Compare against a fixed deadline. Re-reading the clock into the same variable it is compared
  // against made the old condition unconditionally true, so there was no timeout and no backoff.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(5000);

  bool held = false;
  while (!held && std::chrono::steady_clock::now() < deadline) {
    held = IpcMutex_TryLock(m_driverActiveMutex);
    if (!held) {
      // Back off rather than spinning a core flat for the whole timeout.
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  IpcMutex_Unlock(m_driverActiveGuardMutex);

  return held;
}

void CustomShareManager::releaseDriverMutex() {
  // We don't need the guard mutex, as we should the only one that can unlock this mutex.
  IpcMutex_Unlock(m_driverActiveMutex);
}

void CustomShareManager::setGazeStatus(const hmd2_gaze_status_t *pGazeStatus) {
  IpcMutex_Lock(m_gazeStatusMutex);
  m_pBufferData->gazeStatus.set(pGazeStatus);
  m_pBufferData->gazeStatus.counter++;
  IpcMutex_Unlock(m_gazeStatusMutex);
  IpcBroadcast_NotifyAll(m_gazeStatusBroadcast);
}

bool CustomShareManager::getGazeStatus(hmd2_gaze_status_t *pGazeStatus, int *lastCounter, uint32_t timeoutMs) {
  auto start = std::chrono::steady_clock::now();
  while (true) {
    IpcMutex_Lock(m_gazeStatusMutex);
    int currentCounter = m_pBufferData->gazeStatus.counter;
    if (!lastCounter || *lastCounter != currentCounter) {
      m_pBufferData->gazeStatus.get(pGazeStatus);
      if (lastCounter)
        *lastCounter = currentCounter;
      IpcMutex_Unlock(m_gazeStatusMutex);
      return true;
    }
    IpcMutex_Unlock(m_gazeStatusMutex);

    if (timeoutMs == 0) {
      IpcMutex_Lock(m_gazeStatusMutex);
      m_pBufferData->gazeStatus.get(pGazeStatus);
      IpcMutex_Unlock(m_gazeStatusMutex);
      return false;
    }

    auto now = std::chrono::steady_clock::now();
    uint32_t elapsed = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count());
    if (elapsed >= timeoutMs) {
      IpcMutex_Lock(m_gazeStatusMutex);
      m_pBufferData->gazeStatus.get(pGazeStatus);
      IpcMutex_Unlock(m_gazeStatusMutex);
      return false;
    }
    IpcBroadcast_Wait(m_gazeStatusBroadcast, timeoutMs - elapsed);
  }
}

void CustomShareManager::setGazeImage(const unsigned char *pGazeImage, uint32_t size) {
  IpcMutex_Lock(m_gazeImageMutex);
  m_pBufferData->gazeImage.pushToCircularBuffer(pGazeImage, size);
  IpcMutex_Unlock(m_gazeImageMutex);
  IpcBroadcast_NotifyAll(m_gazeImageBroadcast);
}

bool CustomShareManager::getGazeImageCopy(unsigned char *pDest, uint32_t destSize, uint32_t *pOutSize, int *lastCounter, uint32_t timeoutMs) {
  auto start = std::chrono::steady_clock::now();
  while (true) {
    IpcMutex_Lock(m_gazeImageMutex);
    int currentCounter = m_pBufferData->gazeImage.counter;
    if (!lastCounter || *lastCounter != currentCounter) {
      // Copy while still holding the lock: that is the whole point of this entry point over
      // getGazeImageBuffer, which hands out a pointer the producer can recycle underneath the caller.
      const bool complete = m_pBufferData->gazeImage.copyLatest(pDest, destSize, pOutSize);
      if (lastCounter) {
        *lastCounter = currentCounter;
      }
      IpcMutex_Unlock(m_gazeImageMutex);
      return complete;
    }
    IpcMutex_Unlock(m_gazeImageMutex);

    if (timeoutMs == 0) {
      if (pOutSize) {
        *pOutSize = 0;
      }
      return false;
    }

    auto now = std::chrono::steady_clock::now();
    uint32_t elapsed = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count());
    if (elapsed >= timeoutMs) {
      if (pOutSize) {
        *pOutSize = 0;
      }
      return false;
    }
    IpcBroadcast_Wait(m_gazeImageBroadcast, timeoutMs - elapsed);
  }
}

bool CustomShareManager::getGazeImageBuffer(unsigned char **gazeImageBuffer, int *lastCounter, uint32_t timeoutMs) {
  auto start = std::chrono::steady_clock::now();
  while (true) {
    IpcMutex_Lock(m_gazeImageMutex);
    int currentCounter = m_pBufferData->gazeImage.counter;
    if (!lastCounter || *lastCounter != currentCounter) {
      m_pBufferData->gazeImage.getFromCircularBuffer(gazeImageBuffer);
      if (lastCounter)
        *lastCounter = currentCounter;
      IpcMutex_Unlock(m_gazeImageMutex);
      return true;
    }
    IpcMutex_Unlock(m_gazeImageMutex);

    if (timeoutMs == 0) {
      IpcMutex_Lock(m_gazeImageMutex);
      m_pBufferData->gazeImage.getFromCircularBuffer(gazeImageBuffer);
      IpcMutex_Unlock(m_gazeImageMutex);
      return false;
    }

    auto now = std::chrono::steady_clock::now();
    uint32_t elapsed = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count());
    if (elapsed >= timeoutMs) {
      IpcMutex_Lock(m_gazeImageMutex);
      m_pBufferData->gazeImage.getFromCircularBuffer(gazeImageBuffer);
      IpcMutex_Unlock(m_gazeImageMutex);
      return false;
    }
    IpcBroadcast_Wait(m_gazeImageBroadcast, timeoutMs - elapsed);
  }
}

void CustomShareManager::signalPcmUpdate() {
  std::memset(m_pBufferData->pcmLeft, 0, sizeof(m_pBufferData->pcmLeft));
  std::memset(m_pBufferData->pcmRight, 0, sizeof(m_pBufferData->pcmRight));
  IpcBroadcast_NotifyAll(m_pcmBroadcast);
}

void CustomShareManager::readPcm(int slot, unsigned char *pcmLeft, unsigned char *pcmRight) {
  if (pcmLeft)
    std::memcpy(pcmLeft, m_pBufferData->pcmLeft[slot], k_senseChunkSize);
  if (pcmRight)
    std::memcpy(pcmRight, m_pBufferData->pcmRight[slot], k_senseChunkSize);
}

int CustomShareManager::claimSlot() {
  for (int i = 0; i < k_maxSlots; i++) {
    if (IpcMutex_TryLock(m_slotOwnerMutex[i])) {
      return i; // Successfully claimed
    }
  }
  return -1; // No available slots
}

void CustomShareManager::releaseSlot(int slot) {
  if (slot >= 0 && slot < k_maxSlots) {
    IpcMutex_Unlock(m_slotOwnerMutex[slot]);
  }
}

bool CustomShareManager::isSlotAlive(int slot) {
  if (slot < 0 || slot >= k_maxSlots)
    return false;
  if (IpcMutex_TryLock(m_slotOwnerMutex[slot])) {
    IpcMutex_Unlock(m_slotOwnerMutex[slot]);
    return false;
  }
  return true;
}

void CustomShareManager::writePcm(int slot, VRControllerType controllerType, const unsigned char *pcm) {
  if (slot < 0 || slot >= k_maxSlots)
    return;
  if (controllerType == VRControllerType::Left || controllerType == VRControllerType::Both) {
    std::memcpy(m_pBufferData->pcmLeft[slot], pcm, k_senseChunkSize);
  }
  if (controllerType == VRControllerType::Right || controllerType == VRControllerType::Both) {
    std::memcpy(m_pBufferData->pcmRight[slot], pcm, k_senseChunkSize);
  }
}

bool CustomShareManager::waitForPcmUpdate() { return IpcBroadcast_Wait(m_pcmBroadcast, 1000); }

bool CustomShareManager::submitCommand(DriverCommand &command) {
  if (!this->getDriverActive()) {
    return false; // Without the driver, we will not get a response. Return now.
  }

  IpcMutex_Lock(m_commandMutex);
  DriverCommand *ptr = m_pBufferData->commandBuffer.push(command);
  IpcMutex_Unlock(m_commandMutex);

  if (!ptr)
    return false; // Buffer full

  IpcBroadcast_NotifyAll(m_commandBroadcast);

  auto start = std::chrono::steady_clock::now();

  // Acquire, so that whatever the driver wrote into the payload is visible once the flag is observed.
  // This is read without holding m_commandMutex, which is why it needs ordering at all.
  std::atomic_ref<bool> fulfilledFlag(ptr->isFulfilled);

  while (!fulfilledFlag.load(std::memory_order_acquire)) {
    // Allow up to 5 seconds for the command to be fulfilled
    IpcBroadcast_Wait(m_commandBroadcast, 5000);

    auto now = std::chrono::steady_clock::now();
    if (now > start + std::chrono::milliseconds(5000)) {
      break;
    }
  }

  const bool fulfilled = fulfilledFlag.load(std::memory_order_acquire);
  command = *ptr;

  return fulfilled;
}

DriverCommand *CustomShareManager::popCommand(uint32_t timeoutMs) {
  auto start = std::chrono::steady_clock::now();
  while (true) {
    IpcMutex_Lock(m_commandMutex);
    DriverCommand *result = m_pBufferData->commandBuffer.pop();
    IpcMutex_Unlock(m_commandMutex);

    if (result)
      return result;

    auto now = std::chrono::steady_clock::now();
    uint32_t elapsed = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count());
    if (elapsed >= timeoutMs)
      return nullptr;
    IpcBroadcast_Wait(m_commandBroadcast, timeoutMs - elapsed);
  }
}

void CustomShareManager::fulfillCommand(DriverCommand *command) {
  // Release: pair with the acquire in submitCommand so the caller sees the payload we just wrote.
  std::atomic_ref<bool>(command->isFulfilled).store(true, std::memory_order_release);
  IpcBroadcast_NotifyAll(m_commandBroadcast);
}
