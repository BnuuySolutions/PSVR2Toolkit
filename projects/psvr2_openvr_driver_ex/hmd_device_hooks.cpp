#ifdef OPENVR_EXTENSIONS_AVAILABLE
#include "psvr2_openvr_driver/openvr_ex/openvr_ex.h"
#endif

#include "driver_host_proxy.h"
#include "hmd2_gaze.h"
#include "hmd_device_camera.h"
#include "hmd_device_hooks.h"
#include "hmd_driver_loader.h"
#include "hook_lib.h"
#include "vr_settings.h"
#include "util.h"

#include <cmath>
#include <cstdint>
#include <cstring>

namespace psvr2_toolkit {
  void *(*CaesarManager__getInstance)();
  uint64_t(*CaesarManager__getIMUTimestampOffset)(void *thisptr, int64_t *hmdToHostOffset);
  void *(*ShareManager__getInstance)();
  void (*ShareManager__getIntConfig)(void *thisPtr, uint32_t configId, int64_t *outValue);
  void (*ShareManager__setIntConfig)(void *thisPtr, uint32_t configId, int64_t *value);

#ifdef OPENVR_EXTENSIONS_AVAILABLE
  void *g_pOpenVRExHandle = nullptr;
#endif
  vr::VRInputComponentHandle_t eyeTrackingComponent = vr::k_ulInvalidInputComponentHandle;
  int64_t currentBrightness;

  vr::EVRInitError (*sie__psvr2__HmdDevice__Activate)(void *, uint32_t) = nullptr;
  void *(*sie__psvr2__HmdDevice__GetComponent)(void *, char *) = nullptr;

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
    // Camera-related properties:
    // Prop_HasCamera_Bool
    // Prop_NumCameras_Int32
    // Prop_CameraFrameLayout_Int32
    // Prop_CameraStreamFormat_Int32
    // Prop_CameraToHeadTransform_Matrix34
    // Prop_CameraFirmwareVersion_Uint64
    // Prop_CameraFirmwareDescription_String
    // Prop_CameraCompatibilityMode_Int32
    // Prop_CameraToHeadTransforms_Matrix34_Array
    // Prop_CameraWhiteBalance_Vector4_Array
    // Prop_CameraDistortionFunction_Int32_Array
    // Prop_CameraDistortionCoefficients_Float_Array
    // Prop_CameraSupportsCompatibilityModes_Bool
    // Prop_CameraExposureTime_Float
    // Prop_CameraGlobalGain_Float
    // Prop_Hmd_EnableParallelRenderCameras_Bool
    // Prop_HasCameraComponent_Bool

    vr::VRProperties()->SetBoolProperty(ulPropertyContainer, vr::Prop_AllowCameraToggle_Bool, true);
    vr::VRProperties()->SetBoolProperty(ulPropertyContainer, vr::Prop_HasCamera_Bool, true);
    vr::VRProperties()->SetBoolProperty(ulPropertyContainer, vr::Prop_HasCameraComponent_Bool, true);
    vr::VRProperties()->SetInt32Property(ulPropertyContainer, vr::Prop_NumCameras_Int32, 2); // ?

    // Required to make camera work...
    vr::VRProperties()->SetUint64Property(ulPropertyContainer, vr::Prop_FPGAVersion_Uint64, 0x104);
    vr::VRProperties()->SetUint64Property(ulPropertyContainer, vr::Prop_FirmwareVersion_Uint64, 0x56456BA0);
    vr::VRProperties()->SetUint64Property(ulPropertyContainer, vr::Prop_CameraFirmwareVersion_Uint64, 0x200040049);

    vr::HmdMatrix34_t cameraToHeadTransforms[2]
    {
      {
        {
          {  0.96483f, -0.00285f,  0.26284f, -0.05384f },
          { -0.12877f,  0.86660f,  0.48210f, -0.03609f },
          { -0.22915f, -0.49899f,  0.83576f, -0.09910f }
        }
      },
      {
        {
          {  0.96546f,  0.00490f, -0.26052f,  0.02514f },
          {  0.12543f,  0.86764f,  0.48112f, -0.03605f },
          {  0.22840f, -0.49717f,  0.83705f, -0.09950f }
        }
      }
    };

    cameraToHeadTransforms[1] = Util::convert44to34(Util::multiplyMatrix(Util::convert34to44(cameraToHeadTransforms[1]), Util::convert34to44(Util::createTransformMatrixFromEuler({0,0,0}, 2.0f, 0.0f, 0.0f))));
    
    vr::VRProperties()->SetProperty(ulPropertyContainer, vr::Prop_CameraToHeadTransform_Matrix34, &cameraToHeadTransforms, sizeof(vr::HmdMatrix34_t), vr::k_unHmdMatrix34PropertyTag);
    vr::VRProperties()->SetProperty(ulPropertyContainer, vr::Prop_CameraToHeadTransforms_Matrix34_Array, &cameraToHeadTransforms, sizeof(vr::HmdMatrix34_t) * 2, vr::k_unHmdMatrix34PropertyTag);

    vr::VRProperties()->SetInt32Property(ulPropertyContainer, vr::Prop_CameraFrameLayout_Int32, vr::EVRTrackedCameraFrameLayout_Stereo | vr::EVRTrackedCameraFrameLayout_HorizontalLayout);
    vr::VRProperties()->SetInt32Property(ulPropertyContainer, vr::Prop_CameraStreamFormat_Int32, vr::CVS_FORMAT_NV12);

    // 1. Tell SteamVR you are using the F-Theta (Fisheye) mathematical model
    vr::EVRDistortionFunctionType cameraDistortionFunction[2] = {
        vr::VRDistortionFunctionType_Extended_FTheta,
        vr::VRDistortionFunctionType_Extended_FTheta
    };

    vr::VRProperties()->SetProperty(
      ulPropertyContainer,
      vr::Prop_CameraDistortionFunction_Int32_Array,
      &cameraDistortionFunction,
      sizeof(cameraDistortionFunction),
      vr::k_unInt32PropertyTag
    );

    // 2. Insert the coefficients printed by the Python script
    double cameraDistortionCoeffs[2][vr::k_unMaxDistortionFunctionParameters] = {
        { 8.925063, -11.718638, 6.383888, -1.237600, 0.0, 0.0, 0.0, 0.0 },
        { 8.937389, -11.752472, 6.413345, -1.245505, 0.0, 0.0, 0.0, 0.0 }
    };

    vr::VRProperties()->SetProperty(
      ulPropertyContainer,
      vr::Prop_CameraDistortionCoefficients_Float_Array,
      &cameraDistortionCoeffs,
      sizeof(cameraDistortionCoeffs),
      vr::k_unFloatPropertyTag // OpenVR expects standard FloatPropertyTag for the double array
    );

    vr::HmdVector4_t whiteBalance[2] = {
        { 1.0f, 1.0f, 1.0f, 1.0f },
        { 1.0f, 1.0f, 1.0f, 1.0f }
    };
    vr::VRProperties()->SetProperty(ulPropertyContainer, vr::Prop_CameraWhiteBalance_Vector4_Array,
      whiteBalance, sizeof(whiteBalance), vr::k_unHmdVector4PropertyTag);

    vr::VRProperties()->SetFloatProperty(ulPropertyContainer, vr::Prop_CameraExposureTime_Float, 1.0f / 60.0f);
    vr::VRProperties()->SetFloatProperty(ulPropertyContainer, vr::Prop_CameraGlobalGain_Float, 1.0f);

    HmdDeviceCamera* pHmdDeviceCamera = HmdDeviceCamera::Instance();

    vr::EVRInitError eError;
    pHmdDeviceCamera->pVRBlockQueue = (vr::IVRBlockQueue *)vr::VRDriverContext()->GetGenericInterface(vr::IVRBlockQueue_Version, &eError);
    vr::IVRPaths *pVRPaths = (vr::IVRPaths *)vr::VRDriverContext()->GetGenericInterface(vr::IVRPaths_Version, &eError);

    pHmdDeviceCamera->pVRBlockQueue->Create(&pHmdDeviceCamera->blockQueueHandle, "/lighthouse/camera/raw_frames", frameDataSize, 512, 8);

    vr::PathHandle_t widthPath;
    pVRPaths->StringToHandle(&widthPath, "/width");

    int width = IMAGE_WIDTH * 2;
    vr::PathWrite_t widthBatch = {};
    widthBatch.ulPath = widthPath;
    widthBatch.pvBuffer = &width;
    widthBatch.unBufferSize = 4;
    widthBatch.unTag = vr::k_unInt32PropertyTag;
    pVRPaths->WritePathBatch(pHmdDeviceCamera->blockQueueHandle, &widthBatch, 1);

    vr::PathHandle_t heightPath;
    pVRPaths->StringToHandle(&heightPath, "/height");

    int height = IMAGE_HEIGHT;
    vr::PathWrite_t heightBatch = {};
    heightBatch.ulPath = heightPath;
    heightBatch.pvBuffer = &height;
    heightBatch.unBufferSize = 4;
    heightBatch.unTag = vr::k_unInt32PropertyTag;
    pVRPaths->WritePathBatch(pHmdDeviceCamera->blockQueueHandle, &heightBatch, 1);

    vr::PathHandle_t formatPath;
    pVRPaths->StringToHandle(&formatPath, "/format");

    int format = vr::CVS_FORMAT_NV12;
    vr::PathWrite_t formatBatch = {};
    formatBatch.ulPath = formatPath;
    formatBatch.pvBuffer = &format;
    formatBatch.unBufferSize = 4;
    formatBatch.unTag = vr::k_unInt32PropertyTag;
    pVRPaths->WritePathBatch(pHmdDeviceCamera->blockQueueHandle, &formatBatch, 1);

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

  void *sie__psvr2__HmdDevice__GetComponentHook(void *thisptr, char *pchComponentNameAndVersion) {
    if (strcmp(pchComponentNameAndVersion, vr::IVRCameraComponent_Version) == 0) {
      HmdDeviceCamera *pHmdDeviceCamera = HmdDeviceCamera::Instance();
      return pHmdDeviceCamera;
    }

    return sie__psvr2__HmdDevice__GetComponent(thisptr, pchComponentNameAndVersion);
  }

  void (*sie__psvr2__HmdDevice__Deactivate)(void*) = nullptr;
  void sie__psvr2__HmdDevice__DeactivateHook(void* thisptr) {
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

  void HmdDeviceHooks::UpdateGaze(void *pData, size_t dwSize)
  {
    if (eyeTrackingComponent == vr::k_ulInvalidInputComponentHandle)
    {
      return;
    }

    Hmd2GazeState* pGazeState = reinterpret_cast<Hmd2GazeState*>(pData);
    vr::VREyeTrackingData_t eyeTrackingData {};

    bool valid = pGazeState->combined.isGazeDirValid;

    eyeTrackingData.bActive = valid;
    eyeTrackingData.bTracked = valid;
    eyeTrackingData.bValid = valid;

    auto &origin = pGazeState->combined.gazeOriginMm;
    auto &direction = pGazeState->combined.gazeDirNorm;

    eyeTrackingData.vGazeOrigin = vr::HmdVector3_t{ -origin.x / 1000.0f, origin.y / 1000.0f, -origin.z / 1000.0f };
    eyeTrackingData.vGazeTarget = vr::HmdVector3_t{ -direction.x, direction.y, -direction.z };

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

    void *psie__psvr2__HmdDevice__GetComponent = reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x19EC60);
    HookLib::InstallHook(psie__psvr2__HmdDevice__GetComponent,
                  reinterpret_cast<void *>(sie__psvr2__HmdDevice__GetComponentHook),
                  reinterpret_cast<void **>(&sie__psvr2__HmdDevice__GetComponent));
    
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
