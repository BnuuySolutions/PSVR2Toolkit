#pragma once

#include <openvr_driver.h>

namespace psvr2_toolkit {

  class DeviceProviderProxy : public vr::IServerTrackedDeviceProvider {
  public:
    DeviceProviderProxy();

    static DeviceProviderProxy *Instance();

    void SetDeviceProvider(vr::IServerTrackedDeviceProvider *pDeviceProvider);

    /** IServerTrackedDeviceProvider **/

    vr::EVRInitError Init(vr::IVRDriverContext *pDriverContext) override;
    void Cleanup() override;
    const char *const *GetInterfaceVersions() override;
    void RunFrame() override;
    bool ShouldBlockStandbyMode() override;
    void EnterStandby() override;
    void LeaveStandby() override;

  private:
    static DeviceProviderProxy *m_pInstance;

    bool m_initOnce;

    vr::IServerTrackedDeviceProvider *m_pDeviceProvider;

    void InitOnce();

    void InitPatches();
    void InitSystems();
  };

} // psvr2_toolkit
