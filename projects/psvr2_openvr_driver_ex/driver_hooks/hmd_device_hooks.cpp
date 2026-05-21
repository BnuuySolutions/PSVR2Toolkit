#include "driver_interface/caesar_manager.h"
#include "driver_host_proxy.h"
#include "common/hmd2_gaze.h"
#include "hmd_device_hooks.h"
#include "hmd_driver_loader.h"
#include "hook_lib.h"
#include "vr_settings.h"
#include "hmd_math.h"
#include "util.h"

#include <cmath>
#include <cstdint>

namespace psvr2_toolkit {
  void *(*ShareManager__getInstance)();
  void (*ShareManager__getIntConfig)(void *thisPtr, uint32_t configId, int64_t *outValue);
  void (*ShareManager__setIntConfig)(void *thisPtr, uint32_t configId, int64_t *value);

  vr::VRInputComponentHandle_t eyeTrackingComponent = vr::k_ulInvalidInputComponentHandle;
  int64_t currentBrightness;

  vr::EVRInitError(*sie__psvr2__HmdDevice__Activate)(void *, uint32_t) = nullptr;
  vr::EVRInitError sie__psvr2__HmdDevice__ActivateHook(void *thisptr, uint32_t unObjectId) {
    vr::EVRInitError result = sie__psvr2__HmdDevice__Activate(thisptr, unObjectId);
    vr::PropertyContainerHandle_t ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(unObjectId);

    // Sony driver only defines the standard hidden area mesh.
    // OpenVR and OpenXR applications can ask for other types
    // and may end up with broken rendering in some cases
    // (like Unity 6.2+ apps with post processing on).
    // Thanks to Checkerboard for spotting this!
    if (result == vr::VRInitError_None) {
      vr::CVRHiddenAreaHelpers hamHelpers(vr::VRPropertiesRaw());
      
      for (int e = 0; e < 2; ++e) {
        vr::EVREye eye = static_cast<vr::EVREye>(e);
        vr::ETrackedPropertyError err;

        uint32_t vertCount = hamHelpers.GetHiddenArea(eye, vr::k_eHiddenAreaMesh_Standard, nullptr, 0, &err);

        if (vertCount > 0) {
          std::vector<vr::HmdVector2_t> standardVerts(vertCount);
          hamHelpers.GetHiddenArea(eye, vr::k_eHiddenAreaMesh_Standard, standardVerts.data(), vertCount, &err);

          std::vector<vr::HmdVector2_t> perimeter = HmdMath::ExtractInnerHAMPerimeter(standardVerts);

          if (perimeter.size() > 2) {
            // Triangle Fan from optical center
            std::vector<vr::HmdVector2_t> inverseVerts;
            inverseVerts.reserve(perimeter.size() * 3);

            vr::HmdVector2_t center = { 0.5f, 0.5f };
            
            for (size_t i = 0; i < perimeter.size() - 1; ++i) {
              inverseVerts.push_back(center);
              inverseVerts.push_back(perimeter[i]);
              inverseVerts.push_back(perimeter[i + 1]);
            }

            hamHelpers.SetHiddenArea(eye, vr::k_eHiddenAreaMesh_Inverse, inverseVerts.data(), static_cast<uint32_t>(inverseVerts.size()));

            // vrcompositor crashes if these are slightly out of bounds
            for (auto& p : perimeter) {
              p.v[0] = std::clamp(p.v[0], 0.0001f, 0.9999f);
              p.v[1] = std::clamp(p.v[1], 0.0001f, 0.9999f);
            }
            hamHelpers.SetHiddenArea(eye, vr::k_eHiddenAreaMesh_LineLoop, perimeter.data(), static_cast<uint32_t>(perimeter.size()));
          }
        }
      }
    }

    // Tell SteamVR we want the chaperone visibility disabled if we're actually disabling the chaperone.
    if (VRSettings::GetBool(STEAMVR_SETTINGS_DISABLE_CHAPERONE, SETTING_DISABLE_CHAPERONE_DEFAULT_VALUE)) {
      vr::VRProperties()->SetBoolProperty(ulPropertyContainer, vr::Prop_DriverProvidedChaperoneVisibility_Bool, false);
    }

    // Tell SteamVR to allow runtime framerate changes.
    // SteamVR does not allow this feature on AMD GPUs, so this is NVIDIA-only currently.
    vr::VRProperties()->SetBoolProperty(ulPropertyContainer, vr::Prop_DisplaySupportsRuntimeFramerateChange_Bool, true);

    // Tell SteamVR to allow night mode setting.
    vr::VRProperties()->SetBoolProperty(ulPropertyContainer, vr::Prop_DisplayAllowNightMode_Bool, true);

    // Tell SteamVR we support brightness controls.
    vr::VRProperties()->SetBoolProperty(ulPropertyContainer, vr::Prop_DisplaySupportsAnalogGain_Bool, true);
    vr::VRProperties()->SetFloatProperty(ulPropertyContainer, vr::Prop_DisplayMinAnalogGain_Float, 0.0f);
    vr::VRProperties()->SetFloatProperty(ulPropertyContainer, vr::Prop_DisplayMaxAnalogGain_Float, 1.0f);

    // Fill in brightness from PSVR2 config to SteamVR settings key.
    // Also, "analogGain" is stored as a gamma corrected value.
    ShareManager__getIntConfig(ShareManager__getInstance(), 2, &currentBrightness);
    vr::VRSettings()->SetFloat(vr::k_pch_SteamVR_Section, "analogGain", powf(static_cast<float>(currentBrightness) / 31.0f, 2.2f));

    // Set event handler for when brightness ("analogGain") changes.
    DriverHostProxy::Instance()->AddEventHandler([](vr::VREvent_t *event) {
      if (event->eventType == vr::EVREventType::VREvent_SteamVRSectionSettingChanged) {
        float currentFloatBrightness = powf(vr::VRSettings()->GetFloat(vr::k_pch_SteamVR_Section, "analogGain"), 1 / 2.2f);
        if (static_cast<int64_t>(ceilf(currentFloatBrightness * 31.0f)) != currentBrightness)
        {
          currentBrightness = static_cast<int64_t>(ceilf(currentFloatBrightness * 31.0f));
          ShareManager__setIntConfig(ShareManager__getInstance(), 2, &currentBrightness);
        }
      }
    });

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

    return result;
  }

  void (*sie__psvr2__HmdDevice__Deactivate)(void *) = nullptr;
  void sie__psvr2__HmdDevice__DeactivateHook(void *thisptr) {
    sie__psvr2__HmdDevice__Deactivate(thisptr);
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

  void HmdDeviceHooks::UpdateGaze(void *pData, size_t dwSize)
  {
      if (eyeTrackingComponent == vr::k_ulInvalidInputComponentHandle)
      {
        return;
      }

      hmd2_gaze_status_t* pGazeState = reinterpret_cast<hmd2_gaze_status_t*>(pData);
      vr::VREyeTrackingData_t eyeTrackingData {};

    bool valid = pGazeState->wearable.is_gaze_dir_combined_valid;

    eyeTrackingData.bActive = valid;
    eyeTrackingData.bTracked = valid;
    eyeTrackingData.bValid = valid;

    auto &origin = pGazeState->wearable.gaze_origin_combined_mm;
    auto &direction = pGazeState->wearable.gaze_dir_combined_norm;

    eyeTrackingData.vGazeOrigin = vr::HmdVector3_t{ -origin.x / 1000.0f, origin.y / 1000.0f, -origin.z / 1000.0f };
    eyeTrackingData.vGazeTarget = vr::HmdVector3_t{ -direction.x, direction.y, -direction.z };

    int64_t hmdToHostOffset;

    CaesarManager::getSingleton()->getIMUTimestampOffset(&hmdToHostOffset);

    double timeOffset = ((static_cast<int64_t>(pGazeState->wearable.timestamp) + hmdToHostOffset) - GetHostTimestamp()) / 1e6;

    vr::VRDriverInput()->UpdateEyeTrackingComponent(eyeTrackingComponent, &eyeTrackingData, timeOffset);
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

    ShareManager__getInstance = decltype(ShareManager__getInstance)(pHmdDriverLoader->GetBaseAddress() + 0x15bbd0);
    ShareManager__getIntConfig = decltype(ShareManager__getIntConfig)(pHmdDriverLoader->GetBaseAddress() + 0x15d270);
    ShareManager__setIntConfig = decltype(ShareManager__setIntConfig)(pHmdDriverLoader->GetBaseAddress() + 0x15f3d0);
  }

} // psvr2_toolkit
