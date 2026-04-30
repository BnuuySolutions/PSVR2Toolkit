#include "custom_share_manager.h"

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
