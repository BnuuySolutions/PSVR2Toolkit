#pragma once

#include <openvr_driver.h>
#include <cstdint>

namespace sie {
namespace psvr2 {

  class ServerDriver;

  // A faithful reimplementation of the HmdDevice class from the original PS VR2 driver.
  class HmdDevice : public vr::ITrackedDeviceServerDriver, public vr::IVRDisplayComponent {
  public:
    HmdDevice(ServerDriver *serverDriver);

    /** vr::ITrackedDeviceServerDriver **/

    vr::EVRInitError Activate(uint32_t unObjectId) override;
    void Deactivate() override;
    void EnterStandby() override;
    void *GetComponent(const char *pchComponentNameAndVersion) override;
    void DebugRequest(const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize) override;
    vr::DriverPose_t GetPose() override;

  private:
    uint32_t m_unObjectId;

    ServerDriver *m_serverDriver;

    int m_resolutionWidth;
    int m_resolutionHeight;
  };

} // psvr2
} // sie
