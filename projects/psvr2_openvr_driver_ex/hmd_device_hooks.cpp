#ifdef OPENVR_EXTENSIONS_AVAILABLE
#include "psvr2_openvr_driver/openvr_ex/openvr_ex.h"
#endif

#include "driver_host_proxy.h"
#include "hmd2_gaze.h"
#include "hmd_device_hooks.h"
#include "hmd_driver_loader.h"
#include "hook_lib.h"
#include "vr_settings.h"
#include "util.h"

#include <cmath>
#include <cstdint>

namespace psvr2_toolkit {
  void* (*CaesarManager__getInstance)();
  uint64_t (*CaesarManager__getIMUTimestampOffset)(void* thisptr, int64_t* hmdToHostOffset);
  void* (*ShareManager__getInstance)();
  void (*ShareManager__getIntConfig)(void* thisPtr, uint32_t configId, int64_t* outValue);
  void (*ShareManager__setIntConfig)(void* thisPtr, uint32_t configId, int64_t* value);

#ifdef OPENVR_EXTENSIONS_AVAILABLE
  void* g_pOpenVRExHandle = nullptr;
#endif
  vr::VRInputComponentHandle_t eyeTrackingComponent;
  int64_t currentBrightness;

  vr::EVRInitError (*sie__psvr2__HmdDevice__Activate)(void *, uint32_t) = nullptr;
  vr::EVRInitError sie__psvr2__HmdDevice__ActivateHook(void *thisptr, uint32_t unObjectId) {
    vr::EVRInitError result = sie__psvr2__HmdDevice__Activate(thisptr, unObjectId);
    vr::PropertyContainerHandle_t ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(unObjectId);

    // Tell SteamVR we want the chaperone visibility disabled if we're actually disabling the chaperone.
    if (VRSettings::GetBool(STEAMVR_SETTINGS_DISABLE_CHAPERONE, SETTING_DISABLE_CHAPERONE_DEFAULT_VALUE)) {
      vr::VRProperties()->SetBoolProperty(ulPropertyContainer, vr::Prop_DriverProvidedChaperoneVisibility_Bool, false);
    }

    // Tell SteamVR to allow runtime framerate changes.
    // SteamVR does not allow this feature on AMD GPUs, so this is NVIDIA-only currently.
    vr::VRProperties()->SetBoolProperty(ulPropertyContainer, vr::Prop_DisplaySupportsRuntimeFramerateChange_Bool, true);

    // Tell SteamVR to allow night mode setting.
    vr::VRProperties()->SetBoolProperty(ulPropertyContainer, vr::Prop_DisplayAllowNightMode_Bool, true);

    if (VRSettings::GetBool(STEAMVR_SETTINGS_ENABLE_STEAMVR_BRIGHTNESS, SETTING_ENABLE_STEAMVR_BRIGHTNESS_DEFAULT_VALUE)) {
      // Tell SteamVR we support brightness controls.
      vr::VRProperties()->SetBoolProperty(ulPropertyContainer, vr::Prop_DisplaySupportsAnalogGain_Bool, true);
      vr::VRProperties()->SetFloatProperty(ulPropertyContainer, vr::Prop_DisplayMinAnalogGain_Float, 0.0f);
      vr::VRProperties()->SetFloatProperty(ulPropertyContainer, vr::Prop_DisplayMaxAnalogGain_Float, 1.0f);

      // Fill in brightness from PSVR2 config to SteamVR settings key.
      // Also, "analogGain" is stored as a gamma corrected value.
      ShareManager__getIntConfig(ShareManager__getInstance(), 2, &currentBrightness);
      vr::VRSettings()->SetFloat(vr::k_pch_SteamVR_Section, "analogGain", powf(static_cast<float>(currentBrightness) / 31.0f, 2.2f));

      // Set event handler for when brightness ("analogGain") changes.
      DriverHostProxy::Instance()->AddEventHandler([](vr::VREvent_t* event) {
        if (event->eventType == vr::EVREventType::VREvent_SteamVRSectionSettingChanged) {
          float currentFloatBrightness = powf(vr::VRSettings()->GetFloat(vr::k_pch_SteamVR_Section, "analogGain"), 1 / 2.2f);
          if (static_cast<int64_t>(ceilf(currentFloatBrightness * 31.0f)) != currentBrightness)
          {
            currentBrightness = static_cast<int64_t>(ceilf(currentFloatBrightness * 31.0f));
            ShareManager__setIntConfig(ShareManager__getInstance(), 2, &currentBrightness);
          }
        }
        });
    }

    // Tell SteamVR our dashboard scale.
    vr::VRProperties()->SetFloatProperty(ulPropertyContainer, vr::Prop_DashboardScale_Float, .9f);

    vr::VRProperties()->SetBoolProperty(ulPropertyContainer, vr::Prop_SupportsXrEyeGazeInteraction_Bool, true);

    if (vr::VRDriverInput())
    {
        vr::EVRInputError result = (vr::VRDriverInput())->CreateEyeTrackingComponent(ulPropertyContainer, "/eyetracking", &eyeTrackingComponent);
        if (result != vr::VRInputError_None) {
            vr::VRDriverLog()->Log("Failed to create eye tracking component.");
        }
    }
    else
    {
        vr::VRDriverLog()->Log("Failed to get driver input interface. Are you on the latest version of SteamVR?");
    }

#ifdef OPENVR_EXTENSIONS_AVAILABLE
    psvr2_toolkit::openvr_ex::OnHmdActivate(ulPropertyContainer, &g_pOpenVRExHandle);
#endif

    return result;
  }

  void (*sie__psvr2__HmdDevice__Deactivate)(void *) = nullptr;
  void sie__psvr2__HmdDevice__DeactivateHook(void *thisptr) {
    sie__psvr2__HmdDevice__Deactivate(thisptr);

#ifdef OPENVR_EXTENSIONS_AVAILABLE
    if (g_pOpenVRExHandle) {
      psvr2_toolkit::openvr_ex::OnHmdDeactivate(&g_pOpenVRExHandle);
    }
#endif
  }

  inline const int64_t GetHostTimestamp()
  {
    static LARGE_INTEGER frequency{};
    if (frequency.QuadPart == 0)
    {
      QueryPerformanceFrequency(&frequency);
    }

    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);

    return static_cast<int64_t>((static_cast<double>(now.QuadPart) /
        static_cast<double>(frequency.QuadPart)) * 1e6);
  }

  void HmdDeviceHooks::UpdateGaze(void* pData, size_t dwSize)
  {
      Hmd2GazeState* pGazeState = reinterpret_cast<Hmd2GazeState*>(pData);
      vr::VREyeTrackingData_t eyeTrackingData {};

      bool valid = pGazeState->combined.isGazeDirValid;

      eyeTrackingData.bActive = valid;
      eyeTrackingData.bTracked = valid;
      eyeTrackingData.bValid = valid;

      auto& origin = pGazeState->combined.gazeOriginMm;
      auto& direction = pGazeState->combined.gazeDirNorm;

      eyeTrackingData.vGazeOrigin = vr::HmdVector3_t { -origin.x / 1000.0f, origin.y / 1000.0f, -origin.z / 1000.0f };
      eyeTrackingData.vGazeTarget = vr::HmdVector3_t { -direction.x, direction.y, -direction.z };

      int64_t hmdToHostOffset;

      CaesarManager__getIMUTimestampOffset(CaesarManager__getInstance(), &hmdToHostOffset);

      double timeOffset = ((static_cast<int64_t>(pGazeState->combined.timestamp) + hmdToHostOffset) - GetHostTimestamp()) / 1e6;

      (vr::VRDriverInput())->UpdateEyeTrackingComponent(eyeTrackingComponent, &eyeTrackingData, timeOffset);

  #ifdef OPENVR_EXTENSIONS_AVAILABLE
      if (g_pOpenVRExHandle) {
        psvr2_toolkit::openvr_ex::OnHmdUpdate(&g_pOpenVRExHandle, pData, dwSize);
      }
  #endif
  }

  void HmdDeviceHooks::InstallHooks() {
    static HmdDriverLoader *pHmdDriverLoader = HmdDriverLoader::Instance();

    // sie::psvr2::HmdDevice::Activate
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x19D1B0),
                         reinterpret_cast<void *>(sie__psvr2__HmdDevice__ActivateHook),
                         reinterpret_cast<void **>(&sie__psvr2__HmdDevice__Activate));

    // sie::psvr2::HmdDevice::Deactivate
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x19EBF0),
                         reinterpret_cast<void *>(sie__psvr2__HmdDevice__DeactivateHook),
                         reinterpret_cast<void **>(&sie__psvr2__HmdDevice__Deactivate));

    CaesarManager__getInstance = decltype(CaesarManager__getInstance)(pHmdDriverLoader->GetBaseAddress() + 0x124c90);
    CaesarManager__getIMUTimestampOffset = decltype(CaesarManager__getIMUTimestampOffset)(pHmdDriverLoader->GetBaseAddress() + 0x1252e0);
    ShareManager__getInstance = decltype(ShareManager__getInstance)(pHmdDriverLoader->GetBaseAddress() + 0x15bbd0);
    ShareManager__getIntConfig = decltype(ShareManager__getIntConfig)(pHmdDriverLoader->GetBaseAddress() + 0x15d270);
    ShareManager__setIntConfig = decltype(ShareManager__setIntConfig)(pHmdDriverLoader->GetBaseAddress() + 0x15f3d0);
  }

} // psvr2_toolkit
