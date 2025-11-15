#include "custom_share_manager.h"

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
  const char *shareNames[2][2] = {
    {"CUSTOM_SHARE_VRT2_WIN_GAZE_EVT", "CUSTOM_SHARE_VRT2_WIN_GAZE_MTX"}
  };
}
