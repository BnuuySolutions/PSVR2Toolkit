#include "custom_share_manager.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <filesystem>
#include <fstream>
#include "util.h"
#ifdef _WIN32
#include <windows.h>
#endif

void GazeStatus::set(const hmd2_gaze_status_t* pGazeStatus) {
  std::memcpy(&data, pGazeStatus, sizeof(data));
}

void GazeStatus::get(hmd2_gaze_status_t* pGazeStatus) const {
  std::memcpy(pGazeStatus, &data, sizeof(data));
}

void GazeImage::pushToCircularBuffer(const unsigned char* pGazeImage) {
  int index = counter % 8;
  // TODO: size shouldn't be 0x200100?
  std::memcpy(&images[0x200100 * index], pGazeImage, 0x200100);
  counter++;
}

int GazeImage::getFromCircularBuffer(unsigned char** gazeImageBuffer) {
  if (counter == 0) {
    *gazeImageBuffer = nullptr;
    return -1;
  }
  int index = (counter - 1) % 8;
  // TODO: size shouldn't be 0x200100?
  *gazeImageBuffer = &images[0x200100 * index];
  return index;
}

void TriggerEffectBuffer::push(int slot, const TriggerEffectCommandPayload& payload) {
  int next_head = (head + 1) % 256;
  if (next_head == tail) return;
  commands[head].slot = slot;
  commands[head].payload = payload;
  head = next_head;
}

bool TriggerEffectBuffer::pop(TriggerEffectCommand& outCommand) {
  if (head == tail) return false;
  outCommand = commands[tail];
  tail = (tail + 1) % 256;
  return true;
}

DriverCommand* CommandBuffer::push(const DriverCommand& command) {
  int next_head = (head + 1) % 256;
  if (next_head == tail) return nullptr;
  DriverCommand* ptr = &commands[head];
  *ptr = command;
  ptr->isFulfilled = false;
  head = next_head;
  return ptr;
}

DriverCommand* CommandBuffer::pop() {
  if (head == tail) return nullptr;
  DriverCommand* ptr = &commands[tail];
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
  m_gazeStatusBroadcast = CreateIpcBroadcast("CUSTOM_SHARE_VRT2_WIN_GAZE_STATUS_BCAST");
  m_gazeStatusMutex = CreateIpcMutex("CUSTOM_SHARE_VRT2_WIN_GAZE_STATUS_MTX");

  m_gazeImageBroadcast = CreateIpcBroadcast("CUSTOM_SHARE_VRT2_WIN_GAZE_IMAGE_BCAST");
  m_gazeImageMutex = CreateIpcMutex("CUSTOM_SHARE_VRT2_WIN_GAZE_IMAGE_MTX");

  for (int i = 0; i < k_maxSlots; i++) {
    char name[128];
    snprintf(name, sizeof(name), "CUSTOM_SHARE_VRT2_WIN_SLOT_OWNER_MTX_%d", i);
    m_slotOwnerMutex[i] = CreateIpcMutex(name);
  }

  m_pcmBroadcast = CreateIpcBroadcast("CUSTOM_SHARE_VRT2_WIN_PCM_BCAST");
  m_commandBroadcast = CreateIpcBroadcast("CUSTOM_SHARE_VRT2_WIN_CMD_BCAST");

  m_triggerEffectMutex = CreateIpcMutex("CUSTOM_SHARE_VRT2_WIN_TRIGGER_EFFECT_MTX");

  m_commandMutex = CreateIpcMutex("CUSTOM_SHARE_VRT2_WIN_COMMAND_MTX");

  m_sharedMemory = CreateIpcSharedMemory("CUSTOM_SHARE_VRT2_WIN", sizeof(BufferData));
  m_pBufferData = static_cast<BufferData*>(IpcSharedMemory_Map(m_sharedMemory));

#ifdef _WIN32
  try {
    std::filesystem::path temp_folder = GetSystemTempFolder();
    std::filesystem::path path_file = temp_folder / "psvr2tk_capi_path.txt";

    HMODULE hModule = NULL;
    GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                       (LPCSTR)&CustomShareManager::createSingleton, &hModule);
    if (hModule != NULL) {
      char path[MAX_PATH];
      if (GetModuleFileNameA(hModule, path, MAX_PATH) > 0) {
        std::filesystem::path dllPath(path);
        std::filesystem::path capiPath = dllPath.parent_path();

        std::ofstream outFile(path_file);
        if (outFile.is_open()) {
          outFile << capiPath.string();
        }
      }
    }
  } catch (...) {
  }
#endif
}

void CustomShareManager::setGazeStatus(const hmd2_gaze_status_t* pGazeStatus) {
  IpcMutex_Lock(m_gazeStatusMutex);
  m_pBufferData->gazeStatus.set(pGazeStatus);
  m_pBufferData->gazeStatus.counter++;
  IpcMutex_Unlock(m_gazeStatusMutex);
  IpcBroadcast_NotifyAll(m_gazeStatusBroadcast);
}

bool CustomShareManager::getGazeStatus(hmd2_gaze_status_t* pGazeStatus, int* lastCounter, uint32_t timeoutMs) {
  auto start = std::chrono::steady_clock::now();
  while (true) {
    IpcMutex_Lock(m_gazeStatusMutex);
    int currentCounter = m_pBufferData->gazeStatus.counter;
    if (!lastCounter || *lastCounter != currentCounter) {
      m_pBufferData->gazeStatus.get(pGazeStatus);
      if (lastCounter) *lastCounter = currentCounter;
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

void CustomShareManager::setGazeImage(const unsigned char* pGazeImage) {
  IpcMutex_Lock(m_gazeImageMutex);
  m_pBufferData->gazeImage.pushToCircularBuffer(pGazeImage);
  IpcMutex_Unlock(m_gazeImageMutex);
  IpcBroadcast_NotifyAll(m_gazeImageBroadcast);
}

bool CustomShareManager::getGazeImageBuffer(unsigned char** gazeImageBuffer, int* lastCounter, uint32_t timeoutMs) {
  auto start = std::chrono::steady_clock::now();
  while (true) {
    IpcMutex_Lock(m_gazeImageMutex);
    int currentCounter = m_pBufferData->gazeImage.counter;
    if (!lastCounter || *lastCounter != currentCounter) {
      m_pBufferData->gazeImage.getFromCircularBuffer(gazeImageBuffer);
      if (lastCounter) *lastCounter = currentCounter;
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

void CustomShareManager::readPcm(int slot, unsigned char* pcmLeft, unsigned char* pcmRight) {
  if (pcmLeft) std::memcpy(pcmLeft, m_pBufferData->pcmLeft[slot], k_senseChunkSize);
  if (pcmRight) std::memcpy(pcmRight, m_pBufferData->pcmRight[slot], k_senseChunkSize);
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
  if (slot < 0 || slot >= k_maxSlots) return false;
  if (IpcMutex_TryLock(m_slotOwnerMutex[slot])) {
    IpcMutex_Unlock(m_slotOwnerMutex[slot]);
    return false;
  }
  return true;
}

void CustomShareManager::writePcm(int slot, VRControllerType controllerType, const unsigned char* pcm) {
  if (slot < 0 || slot >= k_maxSlots) return;
  if (controllerType == VRControllerType::Left || controllerType == VRControllerType::Both) {
    std::memcpy(m_pBufferData->pcmLeft[slot], pcm, k_senseChunkSize);
  }
  if (controllerType == VRControllerType::Right || controllerType == VRControllerType::Both) {
    std::memcpy(m_pBufferData->pcmRight[slot], pcm, k_senseChunkSize);
  }
}

void CustomShareManager::waitForPcmUpdate() {
  IpcBroadcast_Wait(m_pcmBroadcast, 0xFFFFFFFF);
}

void CustomShareManager::pushTriggerEffect(int slot, const TriggerEffectCommandPayload& payload) {
  IpcMutex_Lock(m_triggerEffectMutex);
  m_pBufferData->triggerEffectBuffer.push(slot, payload);
  IpcMutex_Unlock(m_triggerEffectMutex);
}

bool CustomShareManager::popTriggerEffect(TriggerEffectCommand& outCommand) {
  IpcMutex_Lock(m_triggerEffectMutex);
  bool result = m_pBufferData->triggerEffectBuffer.pop(outCommand);
  IpcMutex_Unlock(m_triggerEffectMutex);
  return result;
}

void CustomShareManager::submitCommand(DriverCommand& command) {
  IpcMutex_Lock(m_commandMutex);
  DriverCommand* ptr = m_pBufferData->commandBuffer.push(command);
  IpcMutex_Unlock(m_commandMutex);

  if (!ptr) return; // Buffer full

  IpcBroadcast_NotifyAll(m_commandBroadcast);

  while (!ptr->isFulfilled) {
    IpcBroadcast_Wait(m_commandBroadcast, 0xFFFFFFFF);
  }

  command = *ptr;
}

DriverCommand* CustomShareManager::popCommand(uint32_t timeoutMs) {
  auto start = std::chrono::steady_clock::now();
  while (true) {
    IpcMutex_Lock(m_commandMutex);
    DriverCommand* result = m_pBufferData->commandBuffer.pop();
    IpcMutex_Unlock(m_commandMutex);

    if (result) return result;
    
    auto now = std::chrono::steady_clock::now();
    uint32_t elapsed = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count());
    if (elapsed >= timeoutMs) return nullptr;
    IpcBroadcast_Wait(m_commandBroadcast, timeoutMs - elapsed);
  }
}

void CustomShareManager::fulfillCommand(DriverCommand* command) {
  command->isFulfilled = true;
  IpcBroadcast_NotifyAll(m_commandBroadcast);
}
