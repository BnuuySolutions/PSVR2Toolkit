#include "ir_tracker_hooks.h"

#include "hmd_driver_loader.h"
#include "utils/hook_lib.h"
#include "util.h"

namespace psvr2_toolkit {

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

void IrTrackerHooks::InstallHooks() {
  static HmdDriverLoader *pHmdDriverLoader = HmdDriverLoader::Instance();
  uintptr_t baseAddress = pHmdDriverLoader->GetBaseAddress();

  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x182E80), reinterpret_cast<void *>(IrTracker__ParseConfigHook),
                       reinterpret_cast<void **>(&IrTracker__ParseConfig));

  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x1999F0), reinterpret_cast<void *>(IrTracker__AssociateLEDsHook),
                       reinterpret_cast<void **>(&IrTracker__AssociateLEDs));
}

} // namespace psvr2_toolkit
