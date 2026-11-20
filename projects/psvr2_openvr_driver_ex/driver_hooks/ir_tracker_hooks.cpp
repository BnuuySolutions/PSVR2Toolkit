#include "ir_tracker_hooks.h"

#include "hmd_driver_loader.h"
#include "utils/hook_lib.h"
#include "util.h"

#include <array>

namespace psvr2_toolkit {

constexpr size_t kLedPayloadSize = 0x139C;
constexpr uint32_t kSenseLedCount = 17;
constexpr uintptr_t kLedPayloadRVA = 0x25EA80; // Model IDs 0, 7, 8, or > 8

/* clang-format off */
// Original LED model positions for right controller but with +1-2mm adjustments along the normals
// to compensate for the plastic shell refracting the light.
// Computed by Beyley via a bundle adjustment of a large controller tracking dataset with CV1.
constexpr float kPssenseRightLeds[kSenseLedCount][3] = {
    {-32.431f,  13.721f, -60.240f},
    { 12.316f, -56.911f,  38.955f},
    { 33.687f, -56.532f,  19.709f},
    { 55.245f, -32.096f,  28.953f},
    { 68.574f, -10.962f,   8.633f},
    { 65.003f,  25.089f,   4.160f},
    { 49.041f,  45.854f,  18.774f},
    {-46.455f, -27.963f,  44.322f},
    {-60.704f, -14.648f,  28.511f},
    {-62.375f,  14.518f,  24.704f},
    {-52.240f,  28.111f,  37.039f},
    {-49.546f,  43.439f,  18.371f},
    {-31.106f,  54.608f,  29.894f},
    {-18.870f,  64.631f,  10.628f},
    {  3.147f,  65.517f,  23.007f},
    { 29.409f,  13.468f, -61.765f},
    { -1.192f,  13.552f, -68.389f},
};
/* clang-format on */

struct IrTrackerModelDesc {
  uint64_t pathOrNull;
  uint8_t flags[8];
  const void *pHeader;
  uint32_t headerSize;
  uint32_t headerPad;
  const void *pPayload;
  uint32_t payloadSize;
  int32_t useInMemory;
};
static_assert(sizeof(IrTrackerModelDesc) == 0x30, "IrTrackerModelDesc size mismatch");

std::atomic<int32_t> IrTrackerHooks::g_controllerLedCount[2] = {0, 0};
std::atomic<uint64_t> IrTrackerHooks::g_opticalFrameIndex[2] = {0, 0};

static void (*IrTracker__AssociateLEDs)(void *pContext, void *pOpticalData) = nullptr;
static void IrTracker__AssociateLEDsHook(void *pContext, void *pOpticalData) {
  uint32_t controllerIdx = *reinterpret_cast<uint32_t *>(reinterpret_cast<uint8_t *>(pContext) + 8);

  if (controllerIdx < 2) {
    uint8_t *pControllerData = reinterpret_cast<uint8_t *>(pOpticalData) + (controllerIdx * 0x5A44);
    int32_t currentLedCount = 0;

    for (int cam = 0; cam < 4; cam++) {
      uint8_t *pCamData = pControllerData + (cam * 0x1688);
      for (int ledId = 0; ledId < 17; ledId++) {
        int16_t blobIndex = *reinterpret_cast<int16_t *>(pCamData + 0x1438 + (ledId * 2));
        if (blobIndex != -1) {
          uint8_t *pBlobData = pCamData + 0x40 + (blobIndex * 0x14);
          int16_t isMatched = *reinterpret_cast<int16_t *>(pBlobData + 8);
          if (isMatched == 1) {
            currentLedCount++;
          }
        }
      }
    }
    IrTrackerHooks::g_controllerLedCount[controllerIdx] = currentLedCount;
    IrTrackerHooks::g_opticalFrameIndex[controllerIdx]++;
  }
  IrTracker__AssociateLEDs(pContext, pOpticalData);
}

static void (*IrTracker__GetModelOrDefault)(int32_t controllerIdx, IrTrackerModelDesc *pModelDesc) = nullptr;
static void IrTracker__GetModelOrDefaultHook(int32_t controllerIdx, IrTrackerModelDesc *pModelDesc) {
  const uint8_t *origPayload = reinterpret_cast<const uint8_t *>(pModelDesc->pPayload);
  uintptr_t baseAddress = HmdDriverLoader::Instance()->GetBaseAddress();
  uintptr_t rva = reinterpret_cast<uintptr_t>(origPayload) - baseAddress;
  if (origPayload && rva == kLedPayloadRVA) {
    std::array<uint8_t, kLedPayloadSize> modifiedPayload;
    std::memcpy(modifiedPayload.data(), origPayload, kLedPayloadSize);

    float *positions = reinterpret_cast<float *>(modifiedPayload.data() + 0x18);

    for (uint32_t i = 0; i < kSenseLedCount; ++i) {
      positions[i * 3 + 0] = kPssenseRightLeds[i][0];
      positions[i * 3 + 1] = kPssenseRightLeds[i][1];
      positions[i * 3 + 2] = kPssenseRightLeds[i][2];
    }

    pModelDesc->pPayload = modifiedPayload.data();
    IrTracker__GetModelOrDefault(controllerIdx, pModelDesc);
    pModelDesc->pPayload = origPayload;
    return;
  }

  Util::DriverLog("[IrTracker__GetModelOrDefaultHook] Ctrl {} was registered to read unknown RVA {}", controllerIdx, rva);
  IrTracker__GetModelOrDefault(controllerIdx, pModelDesc);
}

static int32_t (*IrTracker__ParseConfig)(const void *pConfigStruct, int32_t size, uint8_t *pOutTrackerConfig) = nullptr;
static int32_t IrTracker__ParseConfigHook(const void *pConfigStruct, int32_t size, uint8_t *pOutTrackerConfig) {
  int32_t ret = IrTracker__ParseConfig(pConfigStruct, size, pOutTrackerConfig);

  if (ret == 0 && pOutTrackerConfig && pOutTrackerConfig[0x24] != 0) {
    for (int cam = 0; cam < 4; ++cam) {
      float *pCamIntrinsics = reinterpret_cast<float *>(pOutTrackerConfig + 0x28 + (cam * 0x6C));
      float calibW = pCamIntrinsics[0];
      float &cx = pCamIntrinsics[4];

      if (calibW > 0.0f) {
        float oldCx = cx;
        // So, it seems the LD payload is off by -4px on the x axis on every camera.
        // We will compensate for this by shifting the calibrated focal center the same amount.
        // The LD payload is actually using a binned resolution of 508x508. However, the calibration is still in 1016x1016.
        // Hence why we shift by -8px here instead of -4px because we're dealing with the 1016x1016 calibration.
        cx -= 8.0f;
        Util::DriverLog("[IrTracker__ParseConfigHook] Cam {}: patched cx {:.2f} -> {:.2f} (W={:.0f})", cam, oldCx, cx, calibW);
      }
    }
  }

  return ret;
}

static void (*UkfManager__setImuPose)(uintptr_t ukf, int32_t ctrlIdx, uint32_t poseIdx, const float *offsetMm) = nullptr;
static void UkfManager__setImuPoseHook(uintptr_t ukf, int32_t ctrlIdx, uint32_t poseIdx, const float *offsetMm) {
  UkfManager__setImuPose(ukf, ctrlIdx, poseIdx, offsetMm);

  // The driver uses the libpad library, and the IMU data is rotated along the X axis by default.
  // However, this causes issues in the UKF where there's stuff to help compensate for the IMU clipping at 8G
  // per axis. With the rotation performed outside, the UKF no longer knows when the IMU is actually clipping or not.
  // Two cases can happen, one where the Y and Z axes are close to the 8G max and the rotation makes the
  // acceleration more than 8G in one axis, and another where one axis is maxed out to 8G and is "split" to another axis.
  // The solution is to let the UKF do the rotation instead, as it takes in a pose matrix, not just a position vector.
  if (ukf && ctrlIdx >= 0 && ctrlIdx < 2 && offsetMm) {
    // 50.27 degrees
    constexpr float kCos = 0.63916653f;
    constexpr float kSin = 0.76906836f;

    // Convert to meters, since that's what the UKF works in.
    float ox = offsetMm[0] * 0.001f;
    float oy = offsetMm[1] * 0.001f;
    float oz = offsetMm[2] * 0.001f;

    // Set the IMU pose matrix
    float *M = reinterpret_cast<float *>(ukf + 0x808 + static_cast<size_t>(ctrlIdx) * 0x2218);
    // clang-format off
    M[0]  = 1.0f; M[1]  = 0.0f; M[2]  = 0.0f;  M[3]  = 0.0f;
    M[4]  = 0.0f; M[5]  = kCos; M[6]  = -kSin; M[7]  = 0.0f;
    M[8]  = 0.0f; M[9]  = kSin; M[10] = kCos;  M[11] = 0.0f;
    M[12] = ox;   M[13] = oy;   M[14] = oz;    M[15] = 1.0f;
    // clang-format on

    Util::DriverLog("[UkfManager__setImuPoseHook] Applied rotation to ctrlIdx {}, poseIdx 0x{:X}", ctrlIdx, poseIdx);
  }
}

void IrTrackerHooks::InstallHooks() {
  static HmdDriverLoader *pHmdDriverLoader = HmdDriverLoader::Instance();
  uintptr_t baseAddress = pHmdDriverLoader->GetBaseAddress();

  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x5F1B0), reinterpret_cast<void *>(UkfManager__setImuPoseHook),
                       reinterpret_cast<void **>(&UkfManager__setImuPose));

  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x1999F0), reinterpret_cast<void *>(IrTracker__AssociateLEDsHook),
                       reinterpret_cast<void **>(&IrTracker__AssociateLEDs));

  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x4A340), reinterpret_cast<void *>(IrTracker__GetModelOrDefaultHook),
                       reinterpret_cast<void **>(&IrTracker__GetModelOrDefault));

  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x182E80), reinterpret_cast<void *>(IrTracker__ParseConfigHook),
                       reinterpret_cast<void **>(&IrTracker__ParseConfig));
}

} // namespace psvr2_toolkit
