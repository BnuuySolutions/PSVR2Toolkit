#include "driver_context_proxy.h"

#include "driver_host_proxy.h"

namespace psvr2_toolkit {

  DriverContextProxy *DriverContextProxy::m_pInstance = nullptr;

  DriverContextProxy::DriverContextProxy()
    : m_pDriverContext(nullptr)
  {}

  DriverContextProxy *DriverContextProxy::Instance() {
    if (!m_pInstance) {
      m_pInstance = new DriverContextProxy;
    }

    return m_pInstance;
  }

  void DriverContextProxy::SetDriverContext(vr::IVRDriverContext *pDriverContext) {
    m_pDriverContext = pDriverContext;
  }

  void *DriverContextProxy::GetGenericInterface(const char *pchInterfaceVersion, vr::EVRInitError *peError) {
    void *result = m_pDriverContext->GetGenericInterface(pchInterfaceVersion, peError);

    // Depends on our OpenVR driver SDK version matching the one inside the PS VR2 driver.
    if (strcmp(vr::IVRServerDriverHost_Version, pchInterfaceVersion) == 0) {
      static DriverHostProxy *pDriverHostProxy = DriverHostProxy::Instance();
      pDriverHostProxy->SetDriverHost(static_cast<vr::IVRServerDriverHost *>(result));
      return pDriverHostProxy;
    }

    return result;
  }

  vr::DriverHandle_t DriverContextProxy::GetDriverHandle() {
    return m_pDriverContext->GetDriverHandle();
  }

} // psvr2_toolkit
