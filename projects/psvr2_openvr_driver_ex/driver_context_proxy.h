#pragma once

#include <openvr_driver.h>

namespace psvr2_toolkit {

  class DriverContextProxy : public vr::IVRDriverContext {
  public:
    DriverContextProxy();

    static DriverContextProxy *Instance();

    void SetDriverContext(vr::IVRDriverContext *pDriverContext);

    /** IVRDriverContext **/

    void *GetGenericInterface(const char *pchInterfaceVersion, vr::EVRInitError *peError = nullptr) override;
    vr::DriverHandle_t GetDriverHandle() override;

  private:
    static DriverContextProxy *m_pInstance;

    vr::IVRDriverContext *m_pDriverContext;
  };

} // psvr2_toolkit
