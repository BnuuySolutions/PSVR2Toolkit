#include "device_provider_proxy.h"

#include "caesar_manager_hooks.h"
#include "driver_context_proxy.h"
#include "hmd_device_hooks.h"
#include "hmd_driver_loader.h"
#include "hook_lib.h"
#include "ipc_server.h"
#include "libpad_hooks.h"
#include "trigger_effect_manager.h"
#include "usb_thread_hooks.h"
#include "util.h"
#include "vr_settings.h"

#include <windows.h>
#include "sense_controller.h"

using namespace psvr2_toolkit::ipc;

namespace psvr2_toolkit {

  DeviceProviderProxy *DeviceProviderProxy::m_pInstance = nullptr;

  DeviceProviderProxy::DeviceProviderProxy()
    : m_pDeviceProvider(nullptr)
  {}

  DeviceProviderProxy *DeviceProviderProxy::Instance() {
    if (!m_pInstance) {
      m_pInstance = new DeviceProviderProxy;
    }

    return m_pInstance;
  }

  void DeviceProviderProxy::SetDeviceProvider(vr::IServerTrackedDeviceProvider *pDeviceProvider) {
    m_pDeviceProvider = pDeviceProvider;
  }

  vr::EVRInitError DeviceProviderProxy::Init(vr::IVRDriverContext *pDriverContext) {
#if _DEBUG
    Sleep(8000);
#endif

    if (!HookLib::Initialize()) {
      MessageBoxW(nullptr, L"MinHook initialization failed, please report this to the developers!", L"PlayStation VR2 Toolkit (DriverEx)", MB_ICONERROR | MB_OK);
    }

    VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

    vr::VRDriverLog()->Log(__FUNCTION__); // Log our function name here to show that we're proxied.

    InstallHooks();
    InitializeSystems();

    IpcServer::Instance()->Start();

    static DriverContextProxy *pDriverContextProxy = DriverContextProxy::Instance();
    pDriverContextProxy->SetDriverContext(pDriverContext);

    return m_pDeviceProvider->Init(pDriverContextProxy);
  }

  void DeviceProviderProxy::Cleanup() {
    IpcServer::Instance()->Stop();

    m_pDeviceProvider->Cleanup();
  }

  const char *const *DeviceProviderProxy::GetInterfaceVersions() {
    return m_pDeviceProvider->GetInterfaceVersions();
  }

  void DeviceProviderProxy::RunFrame() {
    m_pDeviceProvider->RunFrame();
  }

  bool DeviceProviderProxy::ShouldBlockStandbyMode() {
    return m_pDeviceProvider->ShouldBlockStandbyMode();
  }

  void DeviceProviderProxy::EnterStandby() {
    m_pDeviceProvider->EnterStandby();
  }

  void DeviceProviderProxy::LeaveStandby() {
    m_pDeviceProvider->LeaveStandby();
  }

  void DeviceProviderProxy::InstallHooks() {
    static HmdDriverLoader *pHmdDriverLoader = HmdDriverLoader::Instance();

    // Remove signature checks.
    HookLib::InstallStubRet0(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x134FF0)); // VrDialogManager::VerifyLibrary

    // If disableSense is enabled, we must disable the overlay and dialog regardless due to a bug.
    if (VRSettings::GetBool(STEAMVR_SETTINGS_DISABLE_OVERLAY, SETTING_DISABLE_OVERLAY_DEFAULT_VALUE) ||
        VRSettings::GetBool(STEAMVR_SETTINGS_DISABLE_SENSE, SETTING_DISABLE_SENSE_DEFAULT_VALUE))
    {
      Util::DriverLog("Disabling PSVR2 overlay...");
      HookLib::InstallStub(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x12F830)); // VrDialogManager::CreateDashboardProcess
    }
    if (VRSettings::GetBool(STEAMVR_SETTINGS_DISABLE_DIALOG, SETTING_DISABLE_DIALOG_DEFAULT_VALUE) ||
        VRSettings::GetBool(STEAMVR_SETTINGS_DISABLE_SENSE, SETTING_DISABLE_SENSE_DEFAULT_VALUE))
    {
      Util::DriverLog("Disabling PSVR2 dialog...");
      HookLib::InstallStub(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x130020)); // VrDialogManager::CreateDialogProcess
    }

    CaesarManagerHooks::InstallHooks();
    HmdDeviceHooks::InstallHooks();
    LibpadHooks::InstallHooks();
    UsbThreadHooks::InstallHooks();
  }

  void DeviceProviderProxy::InitializeSystems() {
    IpcServer::Instance()->Initialize();
    TriggerEffectManager::Instance()->Initialize();
    if (VRSettings::GetBool(STEAMVR_SETTINGS_USE_ENHANCED_HAPTICS, SETTING_USE_TOOLKIT_SYNC_DEFAULT_VALUE)) {
        SenseController::Initialize();
    }
  }

} // psvr2_toolkit
