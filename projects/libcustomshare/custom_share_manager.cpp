#include "custom_share_manager.h"

#include <windows.h>

CustomShareManager *CustomShareManager::m_pInstance = nullptr;

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
  const char *handleNames[1][2] = {
    {"CUSTOM_SHARE_VRT2_WIN_GAZE_STATUS_EVT", "CUSTOM_SHARE_VRT2_WIN_GAZE_STATUS_MTX"}
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
