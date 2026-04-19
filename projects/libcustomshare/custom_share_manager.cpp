#include "custom_share_manager.h"

#include <windows.h>

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
  // If you add more handle names, you must also increase the size of m_handles.
  const char *handleNames[2][2] = {
    {"CUSTOM_SHARE_VRT2_WIN_GAZE_STATUS_EVT", "CUSTOM_SHARE_VRT2_WIN_GAZE_STATUS_MTX"},
    {"CUSTOM_SHARE_VRT2_WIN_GAZE_IMAGE_EVT", "CUSTOM_SHARE_VRT2_WIN_GAZE_IMAGE_MTX"}
  };

  for (size_t i = 0; i < sizeof(handleNames) / sizeof(handleNames[0]); i++) {
    m_handles[i].hEvent = CreateEventA(NULL, TRUE, FALSE, handleNames[i][0]);
    m_handles[i].hMutex = CreateMutexA(NULL, FALSE, handleNames[i][1]);
  }

  m_hFileMap = CreateFileMappingA(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, 0x2000000, "CUSTOM_SHARE_VRT2_WIN");
  m_pBufferData = static_cast<BufferData*>(MapViewOfFile(m_hFileMap, FILE_MAP_WRITE, 0, 0, 0x2000000));
}

void CustomShareManager::setGazeStatus(unsigned char* pGazeStatus) {
  WaitForSingleObject(m_handles[0].hMutex, INFINITE);
  memcpy(&m_pBufferData->gazeStatus, pGazeStatus, 0x148);
  ReleaseMutex(m_handles[0].hMutex);
  SetEvent(m_handles[0].hEvent);
}

void CustomShareManager::getGazeStatus(unsigned char* pGazeStatus) {
  WaitForSingleObject(m_handles[0].hMutex, INFINITE);
  memcpy(pGazeStatus, &m_pBufferData->gazeStatus, 0x148);
  ReleaseMutex(m_handles[0].hMutex);
}

void CustomShareManager::setGazeImage(unsigned char* pGazeImage) {
  WaitForSingleObject(m_handles[1].hMutex, INFINITE);
  memcpy(&m_pBufferData->gazeImage, pGazeImage, 0x200100);
  ReleaseMutex(m_handles[1].hMutex);
  SetEvent(m_handles[1].hEvent);
}

void CustomShareManager::getGazeImage(unsigned char* pGazeImage) {
  WaitForSingleObject(m_handles[1].hMutex, INFINITE);
  memcpy(pGazeImage, &m_pBufferData->gazeImage, 0x200100);
  ReleaseMutex(m_handles[1].hMutex);
}

/*int CustomShareManager::getGazeImageBuffer(unsigned char** gazeImageBuffer, unsigned char** a2) {
  WaitForSingleObject(m_handles[1].hMutex, INFINITE);
  int result = -1;
  int i = 0;
  int gazeImageIndex = 0;
  int gazeImageCount = gazeImageCounter + 1;
  while (1) {
    gazeImageIndex = gazeImageCount % 8;
    if (*(int *)&m_pBufferData->bufferInfo[0x878 * (gazeImageCount % 8)] <= 1) {
      break;
    }
    ++i;
    ++gazeImageCount;
    if (i >= 8) {
      ReleaseMutex(m_handles[1].hMutex);
      return result;
    }
  }
  result = gazeImageCount % 8;
  if (gazeImageIndex >= 0) {
    int bufferInfoOffset = 0x878 * gazeImageIndex;
    *gazeImageBuffer = &m_pBufferData->gazeImage[0x200100 * gazeImageIndex];
    *(int *)&m_pBufferData->bufferInfo[bufferInfoOffset] = 3;
    *a2 = &m_pBufferData->bufferInfo[bufferInfoOffset + 12];
  }
  ReleaseMutex(m_handles[1].hMutex);
  return result;
}*/
