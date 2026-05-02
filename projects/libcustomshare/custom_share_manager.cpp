#include "custom_share_manager.h"

#include <cstdio>
#include <cstring>

void GazeStatus::set(const unsigned char* pGazeStatus) {
  std::memcpy(data, pGazeStatus, sizeof(data));
}

void GazeStatus::get(unsigned char* pGazeStatus) const {
  std::memcpy(pGazeStatus, data, sizeof(data));
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

CustomShareManager *CustomShareManager::m_pInstance = nullptr;
bool CustomShareManager::m_initialized = false;

void CustomShareManager::createSingleton() {
  m_initialized = true;

  CustomShareManager *pInstance = m_pInstance;
  if (!m_pInstance) {
    pInstance = new CustomShareManager;
    m_pInstance = pInstance;
  }

  pInstance->initialize();
}

CustomShareManager *CustomShareManager::getSingleton() {
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

  for (int i = 0; i < MAX_PCM_SLOTS; i++) {
    char name[128];
    snprintf(name, sizeof(name), "CUSTOM_SHARE_VRT2_WIN_PCM_OWNER_MTX_%d", i);
    m_pcmOwnerMutex[i] = CreateIpcMutex(name);

    snprintf(name, sizeof(name), "CUSTOM_SHARE_VRT2_WIN_PCM_EVT_%d", i);
    m_pcmEvent[i] = CreateIpcEvent(name);
  }

  m_sharedMemory = CreateIpcSharedMemory("CUSTOM_SHARE_VRT2_WIN", sizeof(BufferData));
  m_pBufferData = static_cast<BufferData*>(m_sharedMemory->map());
}

void CustomShareManager::setGazeStatus(const unsigned char* pGazeStatus) {
  m_gazeStatusMutex->lock();
  m_pBufferData->gazeStatus.set(pGazeStatus);
  m_gazeStatusMutex->unlock();
  m_gazeStatusEvent->set();
}

void CustomShareManager::getGazeStatus(unsigned char* pGazeStatus) const {
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
  for (int i = 0; i < MAX_PCM_SLOTS; i++) {
    m_pcmEvent[i]->set();
  }
}

void CustomShareManager::readPcm(int slot, unsigned char* pcmLeft, unsigned char* pcmRight) {
  if (pcmLeft) std::memcpy(pcmLeft, m_pBufferData->pcmLeft[slot], PCM_BUFFER_SIZE);
  if (pcmRight) std::memcpy(pcmRight, m_pBufferData->pcmRight[slot], PCM_BUFFER_SIZE);
}

int CustomShareManager::claimPcmSlot() {
  for (int i = 0; i < MAX_PCM_SLOTS; i++) {
    if (m_pcmOwnerMutex[i]->try_lock()) {
      return i; // Successfully claimed
    }
  }
  return -1; // No available slots
}

void CustomShareManager::releasePcmSlot(int slot) {
  if (slot >= 0 && slot < MAX_PCM_SLOTS) {
    m_pcmOwnerMutex[slot]->unlock();
  }
}

void CustomShareManager::writePcm(int slot, const unsigned char* pcmLeft, const unsigned char* pcmRight) {
  if (pcmLeft) std::memcpy(m_pBufferData->pcmLeft[slot], pcmLeft, PCM_BUFFER_SIZE);
  if (pcmRight) std::memcpy(m_pBufferData->pcmRight[slot], pcmRight, PCM_BUFFER_SIZE);
}

void CustomShareManager::waitForPcmUpdate(int slot) {
  m_pcmEvent[slot]->wait();
}
