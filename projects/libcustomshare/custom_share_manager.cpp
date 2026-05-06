#include "custom_share_manager.h"

#include <cstdio>
#include <cstring>
#include <mutex>

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
  m_gazeStatusEvent = CreateIpcEvent("CUSTOM_SHARE_VRT2_WIN_GAZE_STATUS_EVT");
  m_gazeStatusMutex = CreateIpcMutex("CUSTOM_SHARE_VRT2_WIN_GAZE_STATUS_MTX");

  m_gazeImageEvent = CreateIpcEvent("CUSTOM_SHARE_VRT2_WIN_GAZE_IMAGE_EVT");
  m_gazeImageMutex = CreateIpcMutex("CUSTOM_SHARE_VRT2_WIN_GAZE_IMAGE_MTX");

  for (int i = 0; i < k_maxSlots; i++) {
    char name[128];
    snprintf(name, sizeof(name), "CUSTOM_SHARE_VRT2_WIN_SLOT_OWNER_MTX_%d", i);
    m_slotOwnerMutex[i] = CreateIpcMutex(name);

    snprintf(name, sizeof(name), "CUSTOM_SHARE_VRT2_WIN_PCM_EVT_%d", i);
    m_pcmEvent[i] = CreateIpcEvent(name);

    snprintf(name, sizeof(name), "CUSTOM_SHARE_VRT2_WIN_CMD_EVT_%d", i);
    m_commandEvent[i] = CreateIpcEvent(name);
  }

  m_triggerEffectMutex = CreateIpcMutex("CUSTOM_SHARE_VRT2_WIN_TRIGGER_EFFECT_MTX");

  m_commandMutex = CreateIpcMutex("CUSTOM_SHARE_VRT2_WIN_COMMAND_MTX");

  m_sharedMemory = CreateIpcSharedMemory("CUSTOM_SHARE_VRT2_WIN", sizeof(BufferData));
  m_pBufferData = static_cast<BufferData*>(m_sharedMemory->map());
}

void CustomShareManager::setGazeStatus(const hmd2_gaze_status_t* pGazeStatus) {
  m_gazeStatusMutex->lock();
  m_pBufferData->gazeStatus.set(pGazeStatus);
  m_gazeStatusMutex->unlock();
  m_gazeStatusEvent->set();
}

void CustomShareManager::getGazeStatus(hmd2_gaze_status_t* pGazeStatus) const {
  m_gazeStatusMutex->lock();
  m_pBufferData->gazeStatus.get(pGazeStatus);
  m_gazeStatusMutex->unlock();
}

void CustomShareManager::setGazeImage(const unsigned char* pGazeImage) {
  m_gazeImageMutex->lock();
  m_pBufferData->gazeImage.pushToCircularBuffer(pGazeImage);
  m_gazeImageMutex->unlock();
  m_gazeImageEvent->set();
}

int CustomShareManager::getGazeImageBuffer(unsigned char** gazeImageBuffer) {
  m_gazeImageMutex->lock();
  int result = m_pBufferData->gazeImage.getFromCircularBuffer(gazeImageBuffer);
  m_gazeImageMutex->unlock();
  return result;
}

void CustomShareManager::signalPcmUpdate() {
  std::memset(m_pBufferData->pcmLeft, 0, sizeof(m_pBufferData->pcmLeft));
  std::memset(m_pBufferData->pcmRight, 0, sizeof(m_pBufferData->pcmRight));
  for (int i = 0; i < k_maxSlots; i++) {
    m_pcmEvent[i]->set();
  }
}

void CustomShareManager::readPcm(int slot, unsigned char* pcmLeft, unsigned char* pcmRight) {
  if (pcmLeft) std::memcpy(pcmLeft, m_pBufferData->pcmLeft[slot], k_senseChunkSize);
  if (pcmRight) std::memcpy(pcmRight, m_pBufferData->pcmRight[slot], k_senseChunkSize);
}

int CustomShareManager::claimSlot() {
  for (int i = 0; i < k_maxSlots; i++) {
    if (m_slotOwnerMutex[i]->try_lock()) {
      return i; // Successfully claimed
    }
  }
  return -1; // No available slots
}

void CustomShareManager::releaseSlot(int slot) {
  if (slot >= 0 && slot < k_maxSlots) {
    m_slotOwnerMutex[slot]->unlock();
  }
}

bool CustomShareManager::isSlotAlive(int slot) {
  if (slot < 0 || slot >= k_maxSlots) return false;
  if (m_slotOwnerMutex[slot]->try_lock()) {
    m_slotOwnerMutex[slot]->unlock();
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

void CustomShareManager::waitForPcmUpdate(int slot) {
  m_pcmEvent[slot]->wait();
}

void CustomShareManager::pushTriggerEffect(int slot, const TriggerEffectCommandPayload& payload) {
  m_triggerEffectMutex->lock();
  m_pBufferData->triggerEffectBuffer.push(slot, payload);
  m_triggerEffectMutex->unlock();
}

bool CustomShareManager::popTriggerEffect(TriggerEffectCommand& outCommand) {
  m_triggerEffectMutex->lock();
  bool result = m_pBufferData->triggerEffectBuffer.pop(outCommand);
  m_triggerEffectMutex->unlock();
  return result;
}

void CustomShareManager::submitCommand(int slot, DriverCommand& command) {
  if (slot < 0 || slot >= k_maxSlots) return;
  command.slot = slot;

  m_commandMutex->lock();
  DriverCommand* ptr = m_pBufferData->commandBuffer.push(command);
  m_commandMutex->unlock();

  if (!ptr) return; // Buffer full

  while (!ptr->isFulfilled) {
    m_commandEvent[slot]->wait();
  }

  command = *ptr;
}

DriverCommand* CustomShareManager::popCommand() {
  m_commandMutex->lock();
  DriverCommand* result = m_pBufferData->commandBuffer.pop();
  m_commandMutex->unlock();
  return result;
}

void CustomShareManager::fulfillCommand(DriverCommand* command) {
  command->isFulfilled = true;
  if (command->slot >= 0 && command->slot < k_maxSlots) {
    m_commandEvent[command->slot]->set();
  }
}
