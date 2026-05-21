#pragma once

#include <openvr_driver.h>

namespace sie {
namespace psvr2 {

  // A faithful reimplementation of the ServerDriver class from the original PS VR2 driver.
  class ServerDriver : public vr::IServerTrackedDeviceProvider {
  public:
    /** IServerTrackedDeviceProvider **/

    vr::EVRInitError Init(vr::IVRDriverContext *pDriverContext) override;
    void Cleanup() override;
    const char *const *GetInterfaceVersions() override;
    void RunFrame() override;
    bool ShouldBlockStandbyMode() override;
    void EnterStandby() override;
    void LeaveStandby() override;

    void CheckDirectMode(const char *callee, bool *out_EdidTypeChanged = nullptr);

  private:
    float m_preferredRefreshRate;
  };

} // psvr2
} // sie
