#include "driver_interface/caesar_manager.h"
#include "driver_interface/share_manager.h"
#include "driver_host_proxy.h"
#include "gaze_filter.h"
#include "hmd2_gaze.h"
#include "hmd_device_camera.h"
#include "hmd_device_hooks.h"
#include "hmd_driver_loader.h"
#include "utils/hook_lib.h"
#include "openvr_driver.h"
#include "utils/driver_settings.h"
#include "hmd_math.h"
#include "util.h"

#include <cmath>
#include <cstdint>
#include <vector>

namespace psvr2_toolkit {

vr::VRInputComponentHandle_t eyeTrackingComponent = vr::k_ulInvalidInputComponentHandle;
int64_t currentBrightness;

vr::EVRInitError (*sie__psvr2__HmdDevice__Activate)(void *, uint32_t) = nullptr;
vr::EVRInitError sie__psvr2__HmdDevice__ActivateHook(void *thisptr, uint32_t unObjectId) {
  vr::EVRInitError result = sie__psvr2__HmdDevice__Activate(thisptr, unObjectId);
  vr::PropertyContainerHandle_t ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(unObjectId);

  DriverHostProxy::Instance()->SetDevice(DeviceType::HMD, ulPropertyContainer, unObjectId);

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

      if (DriverSettings::GetBool(STEAMVR_SETTINGS_DISABLE_HIDDEN_AREA_MESH, SETTING_DISABLE_HIDDEN_AREA_MESH_DEFAULT_VALUE)) {
        hamHelpers.SetHiddenArea(eye, vr::k_eHiddenAreaMesh_Standard, nullptr, 0);
        hamHelpers.SetHiddenArea(eye, vr::k_eHiddenAreaMesh_Inverse, nullptr, 0);
        hamHelpers.SetHiddenArea(eye, vr::k_eHiddenAreaMesh_LineLoop, nullptr, 0);

        continue;
      }

      uint32_t vertCount = hamHelpers.GetHiddenArea(eye, vr::k_eHiddenAreaMesh_Standard, nullptr, 0, &err);

      if (vertCount > 0) {
        std::vector<vr::HmdVector2_t> standardVerts(vertCount);
        hamHelpers.GetHiddenArea(eye, vr::k_eHiddenAreaMesh_Standard, standardVerts.data(), vertCount, &err);

        std::vector<vr::HmdVector2_t> perimeter = HmdMath::ExtractInnerHAMPerimeter(standardVerts);

        if (perimeter.size() > 2) {
          // Triangle Fan from optical center
          std::vector<vr::HmdVector2_t> inverseVerts;
          inverseVerts.reserve(perimeter.size() * 3);

          vr::HmdVector2_t center = {0.5f, 0.5f};

          for (size_t i = 0; i < perimeter.size() - 1; ++i) {
            inverseVerts.push_back(center);
            inverseVerts.push_back(perimeter[i]);
            inverseVerts.push_back(perimeter[i + 1]);
          }

          hamHelpers.SetHiddenArea(eye, vr::k_eHiddenAreaMesh_Inverse, inverseVerts.data(), static_cast<uint32_t>(inverseVerts.size()));

          // vrcompositor crashes if these are slightly out of bounds
          for (auto &p : perimeter) {
            p.v[0] = std::clamp(p.v[0], 0.0001f, 0.9999f);
            p.v[1] = std::clamp(p.v[1], 0.0001f, 0.9999f);
          }
          hamHelpers.SetHiddenArea(eye, vr::k_eHiddenAreaMesh_LineLoop, perimeter.data(), static_cast<uint32_t>(perimeter.size()));
        }
      }
    }
  }

  // Tell SteamVR we want the chaperone visibility disabled if we're actually disabling the chaperone.
  if (DriverSettings::GetBool(STEAMVR_SETTINGS_DISABLE_CHAPERONE, SETTING_DISABLE_CHAPERONE_DEFAULT_VALUE)) {
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
  ShareManager::GetInstance()->GetIntConfig(SC_ScreenBrightness, &currentBrightness);
  vr::VRSettings()->SetFloat(vr::k_pch_SteamVR_Section, "analogGain", powf(static_cast<float>(currentBrightness) / 31.0f, 2.2f));

  // Set event handler for when brightness ("analogGain") changes.
  DriverHostProxy::Instance()->AddEventHandler([](vr::VREvent_t *event) {
    if (event->eventType == vr::EVREventType::VREvent_SteamVRSectionSettingChanged) {
      float currentFloatBrightness = powf(vr::VRSettings()->GetFloat(vr::k_pch_SteamVR_Section, "analogGain"), 1 / 2.2f);
      if (static_cast<int64_t>(ceilf(currentFloatBrightness * 31.0f)) != currentBrightness) {
        currentBrightness = static_cast<int64_t>(ceilf(currentFloatBrightness * 31.0f));
        ShareManager::GetInstance()->SetIntConfig(SC_ScreenBrightness, &currentBrightness);
      }
    }
  });

  bool enableCamera = vr::VRSettings()->GetBool(vr::k_pch_Camera_Section, vr::k_pch_Camera_EnableCamera_Bool);

  if (!Util::IsRunningOnWine() || enableCamera) {
    HmdDeviceCamera *pHmdDeviceCamera = HmdDeviceCamera::Instance();
    pHmdDeviceCamera->LoadCalibration();

    vr::VRProperties()->SetBoolProperty(ulPropertyContainer, vr::Prop_AllowCameraToggle_Bool, true);
    vr::VRProperties()->SetBoolProperty(ulPropertyContainer, vr::Prop_HasCamera_Bool, true);
    vr::VRProperties()->SetBoolProperty(ulPropertyContainer, vr::Prop_HasCameraComponent_Bool, true);
    vr::VRProperties()->SetInt32Property(ulPropertyContainer, vr::Prop_NumCameras_Int32, 2); // ?

    // Required to make camera work...
    vr::VRProperties()->SetUint64Property(ulPropertyContainer, vr::Prop_FPGAVersion_Uint64, 0x104);
    vr::VRProperties()->SetUint64Property(ulPropertyContainer, vr::Prop_FirmwareVersion_Uint64, 0x56456BA0);
    vr::VRProperties()->SetUint64Property(ulPropertyContainer, vr::Prop_CameraFirmwareVersion_Uint64, 0x200040049);

    vr::HmdMatrix34_t cameraToHeadTransforms[2]{};

    ShareManager *pShareManager = ShareManager::GetInstance();
    if (pShareManager) {
      uint8_t d0cBuffer[0x800] = {0};
      uint32_t counter = 0;
      pShareManager->ReadCalib_0xd0c(d0cBuffer, &counter);

      if (counter > 0) {
        float *r0_raw = reinterpret_cast<float *>(d0cBuffer + 0x6f0);
        float *r1_raw = reinterpret_cast<float *>(d0cBuffer + 0x7dc);

        // TODO: figure out what values in ConfigManager could give us these numbers.
        const float headOffsetLeft[3] = {-0.04f, -0.03309f, -0.0935f};
        const float headOffsetRight[3] = {0.04f, -0.03309f, -0.0935f};

        const float radX = -15.0f * std::numbers::pi / 180.0f;
        const float cosX = std::cos(radX);
        const float sinX = std::sin(radX);
        const float rx15[9] = {1.0f, 0.0f, 0.0f, 0.0f, cosX, -sinX, 0.0f, sinX, cosX};

        auto applyPitchAndOffset = [&](const float *r_raw, const float offset[3], const float *rotation, vr::HmdMatrix34_t &outTransform) {
          for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
              outTransform.m[i][j] = rotation[i * 3 + 0] * r_raw[0 * 3 + j] + rotation[i * 3 + 1] * r_raw[1 * 3 + j] + rotation[i * 3 + 2] * r_raw[2 * 3 + j];
            }
            outTransform.m[i][3] = offset[i];
          }
        };

        applyPitchAndOffset(r0_raw, headOffsetLeft, rx15, cameraToHeadTransforms[0]);
        applyPitchAndOffset(r1_raw, headOffsetRight, rx15, cameraToHeadTransforms[1]);
      }
    }

    vr::VRProperties()->SetProperty(ulPropertyContainer, vr::Prop_CameraToHeadTransform_Matrix34, &cameraToHeadTransforms, sizeof(vr::HmdMatrix34_t),
                                    vr::k_unHmdMatrix34PropertyTag);
    vr::VRProperties()->SetProperty(ulPropertyContainer, vr::Prop_CameraToHeadTransforms_Matrix34_Array, &cameraToHeadTransforms, sizeof(vr::HmdMatrix34_t) * 2,
                                    vr::k_unHmdMatrix34PropertyTag);

    vr::VRProperties()->SetInt32Property(ulPropertyContainer, vr::Prop_CameraFrameLayout_Int32,
                                         vr::EVRTrackedCameraFrameLayout_Stereo | vr::EVRTrackedCameraFrameLayout_HorizontalLayout);
    vr::VRProperties()->SetInt32Property(ulPropertyContainer, vr::Prop_CameraStreamFormat_Int32, vr::CVS_FORMAT_NV12);

    vr::EVRDistortionFunctionType cameraDistortionFunction[2] = {vr::VRDistortionFunctionType_Extended_FTheta, vr::VRDistortionFunctionType_Extended_FTheta};

    vr::VRProperties()->SetProperty(ulPropertyContainer, vr::Prop_CameraDistortionFunction_Int32_Array, &cameraDistortionFunction,
                                    sizeof(cameraDistortionFunction), vr::k_unInt32PropertyTag);

    float cameraDistortionCoeffs[2][vr::k_unMaxDistortionFunctionParameters];
    memcpy(cameraDistortionCoeffs, pHmdDeviceCamera->fittedCoefficients, sizeof(cameraDistortionCoeffs));
    vr::VRProperties()->SetProperty(ulPropertyContainer, vr::Prop_CameraDistortionCoefficients_Float_Array, cameraDistortionCoeffs,
                                    sizeof(cameraDistortionCoeffs), vr::k_unDoublePropertyTag);

    vr::HmdVector4_t whiteBalance[2] = {{1.0f, 1.0f, 1.0f, 1.0f}, {1.0f, 1.0f, 1.0f, 1.0f}};
    vr::VRProperties()->SetProperty(ulPropertyContainer, vr::Prop_CameraWhiteBalance_Vector4_Array, whiteBalance, sizeof(whiteBalance),
                                    vr::k_unHmdVector4PropertyTag);

    vr::VRProperties()->SetFloatProperty(ulPropertyContainer, vr::Prop_CameraExposureTime_Float, 1.0f / 59.94f);
    vr::VRProperties()->SetFloatProperty(ulPropertyContainer, vr::Prop_CameraGlobalGain_Float, 1.0f);

    vr::EVRInitError eError;
    pHmdDeviceCamera->pVRBlockQueue = (vr::IVRBlockQueue *)vr::VRDriverContext()->GetGenericInterface(vr::IVRBlockQueue_Version, &eError);
    vr::IVRPaths *pVRPaths = (vr::IVRPaths *)vr::VRDriverContext()->GetGenericInterface(vr::IVRPaths_Version, &eError);

    pHmdDeviceCamera->pVRBlockQueue->Create(&pHmdDeviceCamera->blockQueueHandle, "/lighthouse/camera/raw_frames", frameDataSize, 512, 8);

    int32_t width = IMAGE_WIDTH * 2;
    vr::WritePathProperty(pVRPaths, pHmdDeviceCamera->blockQueueHandle, "/width", width);

    int32_t height = IMAGE_HEIGHT;
    vr::WritePathProperty(pVRPaths, pHmdDeviceCamera->blockQueueHandle, "/height", height);

    int32_t format = vr::CVS_FORMAT_NV12;
    vr::WritePathProperty(pVRPaths, pHmdDeviceCamera->blockQueueHandle, "/format", format);
  }

  // Tell SteamVR our dashboard scale.
  vr::VRProperties()->SetFloatProperty(ulPropertyContainer, vr::Prop_DashboardScale_Float, .9f);

  // Only advertise eye gaze when the pipeline feeding it is actually running. With disableGaze set,
  // CaesarManagerHooks never installs the gaze hooks, so the component would exist and never update --
  // applications would light up gaze-driven UI that then never moves, which is worse for them than
  // being told there is no eye tracking at all.
  if (DriverSettings::IsGazeEnabled()) {
    vr::VRProperties()->SetBoolProperty(ulPropertyContainer, vr::Prop_SupportsXrEyeGazeInteraction_Bool, true);

    if (vr::VRDriverInput()) {
      // Deliberately not named 'result': that would shadow the EVRInitError this function returns.
      vr::EVRInputError eyeTrackingError = (vr::VRDriverInput())->CreateEyeTrackingComponent(ulPropertyContainer, "/eyetracking", &eyeTrackingComponent);
      if (eyeTrackingError != vr::VRInputError_None) {
        vr::VRDriverLog()->Log("Failed to create eye tracking component.");
      }
    } else {
      vr::VRDriverLog()->Log("Failed to get driver input interface. Are you on the latest version of SteamVR?");
    }
  } else {
    Util::DriverLog("[Gaze] disableGaze is set; eye tracking will not be advertised.");
  }

  return result;
}

void (*sie__psvr2__HmdDevice__Deactivate)(void *) = nullptr;
void sie__psvr2__HmdDevice__DeactivateHook(void *thisptr) { sie__psvr2__HmdDevice__Deactivate(thisptr); }

void *(*sie__psvr2__HmdDevice__GetComponent)(void *, char *) = nullptr;
void *sie__psvr2__HmdDevice__GetComponentHook(void *thisptr, char *pchComponentNameAndVersion) {
  bool enableCamera = vr::VRSettings()->GetBool(vr::k_pch_Camera_Section, vr::k_pch_Camera_EnableCamera_Bool);

  if (!Util::IsRunningOnWine() || enableCamera) {
    if (strcmp(pchComponentNameAndVersion, vr::IVRCameraComponent_Version) == 0) {
      HmdDeviceCamera *pHmdDeviceCamera = HmdDeviceCamera::Instance();
      return pHmdDeviceCamera;
    }
  }

  return sie__psvr2__HmdDevice__GetComponent(thisptr, pchComponentNameAndVersion);
}

inline const int64_t GetHostTimestamp() {
  static LARGE_INTEGER frequency{};
  if (frequency.QuadPart == 0) {
    QueryPerformanceFrequency(&frequency);
  }

  LARGE_INTEGER now;
  QueryPerformanceCounter(&now);

  return static_cast<int64_t>((static_cast<double>(now.QuadPart) / static_cast<double>(frequency.QuadPart)) * 1e6);
}

// Smoothing for the SteamVR gaze path only. The C API and the legacy IPC server keep delivering raw
// samples: clients doing their own processing need the unfiltered stream, and silently filtering it
// would be a breaking behavioural change for them.
//
// UpdateGaze is only ever called from the single gaze USB thread, so this needs no locking.
static GazeFilter s_gazeFilter;

static bool IsGazeFilterEnabled() {
  static const bool enabled = DriverSettings::GetBool(STEAMVR_SETTINGS_GAZE_FILTER_ENABLED, SETTING_GAZE_FILTER_ENABLED_DEFAULT_VALUE);
  return enabled;
}

// Reads the filter tuning once, on the first gaze frame. Settings are not re-read afterwards, matching
// how the rest of the driver treats them.
static GazeFilter &ConfiguredGazeFilter() {
  static const bool configured = [] {
    GazeFilterConfig config;
    config.minCutoffHz = DriverSettings::GetFloat(STEAMVR_SETTINGS_GAZE_FILTER_MIN_CUTOFF, SETTING_GAZE_FILTER_MIN_CUTOFF_DEFAULT_VALUE);
    config.beta = DriverSettings::GetFloat(STEAMVR_SETTINGS_GAZE_FILTER_BETA, SETTING_GAZE_FILTER_BETA_DEFAULT_VALUE);
    config.saccadeVelocityDegPerSec =
        DriverSettings::GetFloat(STEAMVR_SETTINGS_GAZE_FILTER_SACCADE_DEG_PER_SEC, SETTING_GAZE_FILTER_SACCADE_DEG_PER_SEC_DEFAULT_VALUE);
    config.blinkHoldUs = static_cast<int64_t>(DriverSettings::GetInt32(STEAMVR_SETTINGS_GAZE_BLINK_HOLD_MS, SETTING_GAZE_BLINK_HOLD_MS_DEFAULT_VALUE)) * 1000;

    s_gazeFilter.Configure(config);

    Util::DriverLog("[Gaze] Filter enabled: minCutoff={} Hz, beta={} Hz per deg/s, saccade={} deg/s, blinkHold={} us.", config.minCutoffHz, config.beta,
                    config.saccadeVelocityDegPerSec, config.blinkHoldUs);
    return true;
  }();
  (void)configured;
  return s_gazeFilter;
}

// Distance at which to place the fixation point, in metres.
//
// The headset reports convergence distance in the foveated block, while the validity flag for it lives
// in the wearable block. Falls back to a configured default when it is unusable, and clamps so that a
// bad reading cannot put the fixation point behind the viewer or at infinity.
static float GetFixationDistanceMeters(const hmd2_gaze_status_t &gazeState) {
  static const float s_defaultDistanceM =
      DriverSettings::GetFloat(STEAMVR_SETTINGS_GAZE_DEFAULT_FIXATION_DISTANCE_M, SETTING_GAZE_DEFAULT_FIXATION_DISTANCE_M_DEFAULT_VALUE);

  constexpr float k_minFixationDistanceM = 0.1f;
  constexpr float k_maxFixationDistanceM = 20.0f;

  if (gazeState.wearable.is_convergence_distance_valid != HMD2_GAZE_BOOL_TRUE) {
    return s_defaultDistanceM;
  }

  const float distanceM = gazeState.foveated.convergence_distance_mm / 1000.0f;
  if (!std::isfinite(distanceM) || distanceM < k_minFixationDistanceM || distanceM > k_maxFixationDistanceM) {
    return s_defaultDistanceM;
  }

  return distanceM;
}

void HmdDeviceHooks::UpdateGaze(void *pData, size_t dwSize) {
  if (eyeTrackingComponent == vr::k_ulInvalidInputComponentHandle) {
    return;
  }

  hmd2_gaze_status_t *pGazeState = reinterpret_cast<hmd2_gaze_status_t *>(pData);
  vr::VREyeTrackingData_t eyeTrackingData{};

  const bool dirValid = pGazeState->wearable.is_gaze_dir_combined_valid == HMD2_GAZE_BOOL_TRUE;
  const bool originValid = pGazeState->wearable.is_gaze_origin_combined_valid == HMD2_GAZE_BOOL_TRUE;

  auto &origin = pGazeState->wearable.gaze_origin_combined_mm;
  auto &direction = pGazeState->wearable.gaze_dir_combined_norm;

  // Smooth in headset space, before the axis flip, so the filter sees the same frame the hardware
  // reports in. The blink hold can keep this valid for a short while after dirValid has gone false.
  GazeFilterVec3 filteredDirection{direction.x, direction.y, direction.z};
  bool sampleValid = dirValid;

  if (IsGazeFilterEnabled()) {
    const GazeFilterOutput filtered = ConfiguredGazeFilter().Update(filteredDirection, dirValid, static_cast<int64_t>(pGazeState->wearable.timestamp));
    filteredDirection = filtered.direction;
    sampleValid = filtered.valid;
  }

  // The eyeball centre barely moves, so holding the last good origin across a blink is what makes the
  // direction hold above usable: without it the ray would be valid but originless.
  static vr::HmdVector3_t s_lastValidOriginMeters{};
  static bool s_haveValidOrigin = false;

  vr::HmdVector3_t originMeters{-origin.x / 1000.0f, origin.y / 1000.0f, -origin.z / 1000.0f};
  if (originValid) {
    s_lastValidOriginMeters = originMeters;
    s_haveValidOrigin = true;
  } else if (s_haveValidOrigin) {
    originMeters = s_lastValidOriginMeters;
  }

  // bActive means "the tracker is running", which stays true across a momentary dropout. bTracked and
  // bValid describe this particular sample, and a sample is only usable if both halves of the ray are:
  // the origin feeds the target below, so a bad origin would corrupt both fields.
  eyeTrackingData.bActive = dirValid;
  eyeTrackingData.bTracked = sampleValid && s_haveValidOrigin;
  eyeTrackingData.bValid = sampleValid && s_haveValidOrigin;

  // SteamVR's x and z run opposite to the headset's.
  const vr::HmdVector3_t directionMeters{-filteredDirection.x, filteredDirection.y, -filteredDirection.z};

  // vGazeTarget is a fixation POINT in the same space as vGazeOrigin, not a direction. Writing the
  // unit direction vector here left every consumer computing normalize(target - origin) with a ray
  // skewed by the ~30 mm eye offset against a 1 m vector -- on the order of two degrees of error.
  const float fixationDistanceM = GetFixationDistanceMeters(*pGazeState);

  eyeTrackingData.vGazeOrigin = originMeters;
  eyeTrackingData.vGazeTarget =
      vr::HmdVector3_t{originMeters.v[0] + directionMeters.v[0] * fixationDistanceM, originMeters.v[1] + directionMeters.v[1] * fixationDistanceM,
                       originMeters.v[2] + directionMeters.v[2] * fixationDistanceM};

  int64_t hmdToHostOffset;

  CaesarManager::getSingleton()->getIMUTimestampOffset(&hmdToHostOffset);

  double timeOffset = ((static_cast<int64_t>(pGazeState->wearable.timestamp) + hmdToHostOffset) - GetHostTimestamp()) / 1e6;

  (vr::VRDriverInput())->UpdateEyeTrackingComponent(eyeTrackingComponent, &eyeTrackingData, timeOffset);
}

void HmdDeviceHooks::InstallHooks() {
  static HmdDriverLoader *pHmdDriverLoader = HmdDriverLoader::Instance();

  // sie::psvr2::HmdDevice::Activate
  HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x19D1B0), reinterpret_cast<void *>(sie__psvr2__HmdDevice__ActivateHook),
                       reinterpret_cast<void **>(&sie__psvr2__HmdDevice__Activate));

  // sie::psvr2::HmdDevice::Deactivate
  HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x19EBF0), reinterpret_cast<void *>(sie__psvr2__HmdDevice__DeactivateHook),
                       reinterpret_cast<void **>(&sie__psvr2__HmdDevice__Deactivate));

  // sie::psvr2::HmdDevice::GetComponent
  HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x19EC60),
                       reinterpret_cast<void *>(sie__psvr2__HmdDevice__GetComponentHook), reinterpret_cast<void **>(&sie__psvr2__HmdDevice__GetComponent));
}

} // namespace psvr2_toolkit
