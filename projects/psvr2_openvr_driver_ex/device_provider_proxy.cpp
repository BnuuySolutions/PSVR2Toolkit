#include "device_provider_proxy.h"

#include "config.h"
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

  DeviceProviderProxy* DeviceProviderProxy::m_pInstance = nullptr;

  DeviceProviderProxy::DeviceProviderProxy()
    : m_initOnce(false)
    , m_pDeviceProvider(nullptr)
  {
  }

  DeviceProviderProxy* DeviceProviderProxy::Instance() {
    if (!m_pInstance) {
      m_pInstance = new DeviceProviderProxy;
    }

    return m_pInstance;
  }

  void DeviceProviderProxy::SetDeviceProvider(vr::IServerTrackedDeviceProvider* pDeviceProvider) {
    m_pDeviceProvider = pDeviceProvider;
  }

  vr::EVRInitError DeviceProviderProxy::Init(vr::IVRDriverContext* pDriverContext) {
#if _DEBUG
    Sleep(8000);
#endif

    VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

    if (!m_initOnce) {
      InitOnce();
      m_initOnce = true;
    }

    IpcServer::Instance()->Start();

    static DriverContextProxy* pDriverContextProxy = DriverContextProxy::Instance();
    pDriverContextProxy->SetDriverContext(pDriverContext);

    return m_pDeviceProvider->Init(pDriverContextProxy);
  }

  void DeviceProviderProxy::Cleanup() {
    IpcServer::Instance()->Stop();

    if (VRSettings::GetBool(STEAMVR_SETTINGS_USE_ENHANCED_HAPTICS, SETTING_USE_TOOLKIT_SYNC_DEFAULT_VALUE)) {
      SenseController::Destroy();
    }

    m_pDeviceProvider->Cleanup();

    VR_CLEANUP_SERVER_DRIVER_CONTEXT();
  }

  const char* const* DeviceProviderProxy::GetInterfaceVersions() {
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

  void DeviceProviderProxy::InitOnce() {
    static bool isRunningOnWine = Util::IsRunningOnWine();

    // Log ourselves here to show that we're proxied.
    Util::DriverLog("PlayStation VR2 Toolkit - v{}.{}.{} [{}]", DRIVER_VERSION_MAJOR, DRIVER_VERSION_MINOR, DRIVER_VERSION_PATCH, DRIVER_VERSION_BRANCH);
#if DRIVER_IS_PRERELEASE
    Util::DriverLog("You are using a pre-release build of PlayStation VR2 Toolkit, please report any issues that may occur to the developers!");
#elif DRIVER_IS_EXPERIMENTAL
    Util::DriverLog("You are using an experimental build of PlayStation VR2 Toolkit, please report any issues that may occur to the developers!");
#endif

    if (!HookLib::Initialize()) {
      MessageBoxW(nullptr, L"MinHook initialization failed, please report this to the developers!", L"PlayStation VR2 Toolkit (DriverEx)", MB_ICONERROR | MB_OK);
    }

    if (isRunningOnWine) {
      Util::DriverLog("PlayStation VR2 Toolkit has detected itself running on Wine, compatibility patches will be applied.");
    }

    InitPatches();
    InitSystems();
  }

  void DeviceProviderProxy::InitPatches() {
    static HmdDriverLoader* pHmdDriverLoader = HmdDriverLoader::Instance();
    static bool isRunningOnWine = Util::IsRunningOnWine();

    // Remove signature checks.
    INSTALL_STUB_RET0(reinterpret_cast<void*>(pHmdDriverLoader->GetBaseAddress() + 0x134FF0)); // VrDialogManager::VerifyLibrary

    // If disableSense is enabled, we must disable the overlay and dialog regardless due to a bug.
    if (VRSettings::GetBool(STEAMVR_SETTINGS_DISABLE_OVERLAY, SETTING_DISABLE_OVERLAY_DEFAULT_VALUE) ||
      VRSettings::GetBool(STEAMVR_SETTINGS_DISABLE_SENSE, SETTING_DISABLE_SENSE_DEFAULT_VALUE) ||
      isRunningOnWine)
    {
      INSTALL_STUB(reinterpret_cast<void*>(pHmdDriverLoader->GetBaseAddress() + 0x12F830)); // VrDialogManager::CreateDashboardProcess
    }
    if (VRSettings::GetBool(STEAMVR_SETTINGS_DISABLE_DIALOG, SETTING_DISABLE_DIALOG_DEFAULT_VALUE) ||
      VRSettings::GetBool(STEAMVR_SETTINGS_DISABLE_SENSE, SETTING_DISABLE_SENSE_DEFAULT_VALUE) ||
      isRunningOnWine)
    {
      INSTALL_STUB(reinterpret_cast<void*>(pHmdDriverLoader->GetBaseAddress() + 0x130020)); // VrDialogManager::CreateDialogProcess
    }

    CaesarManagerHooks::InstallHooks();
    HmdDeviceHooks::InstallHooks();
    LibpadHooks::InstallHooks();
    UsbThreadHooks::InstallHooks();
  }

  void DeviceProviderProxy::InitSystems() {
    IpcServer::Instance()->Initialize();
    TriggerEffectManager::Instance()->Initialize();
    if (VRSettings::GetBool(STEAMVR_SETTINGS_USE_ENHANCED_HAPTICS, SETTING_USE_TOOLKIT_SYNC_DEFAULT_VALUE)) {
      SenseController::Initialize();
    }
  }

} // psvr2_toolkit
