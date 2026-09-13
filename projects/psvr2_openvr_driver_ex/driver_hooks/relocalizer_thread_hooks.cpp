#include "relocalizer_thread_hooks.h"

#include "hmd_driver_loader.h"
#include "utils/hook_lib.h"
#include "util.h"

#include <cstdint>

namespace psvr2_toolkit {

int (*RelocalizerThread__RunLocalizer)(void *, void *, void *, void *, void *) = nullptr;
int RelocalizerThread__RunLocalizerHook(void *pTrackerContext, void *pRPPacket, void *pLocalizerResult, void *pRelocPreOutput, void *pOutStatus) {
  int result = RelocalizerThread__RunLocalizer(pTrackerContext, pRPPacket, pLocalizerResult, pRelocPreOutput, pOutStatus);

  if (result == 0 && pRelocPreOutput != nullptr) {
    uint8_t *pOutputBytes = reinterpret_cast<uint8_t *>(pRelocPreOutput);

    if (pOutputBytes[4] != 0) {
      int32_t camId = -1;
      if (pRPPacket != nullptr) {
        camId = *reinterpret_cast<int32_t *>(reinterpret_cast<uint8_t *>(pRPPacket) + 0x9AE68);
      }

      if (camId == 2 || camId == 3) {
        if (pOutputBytes[4] != 0) {
          pOutputBytes[4] = 0; // Mark as invalid/untracked
          Util::DriverLog("Attempted to run localizer on camera {}.\n", camId);
        }
        return result;
      }
    }
  }

  return result;
}

void RelocalizerThreadHooks::InstallHooks() {
  static HmdDriverLoader *pHmdDriverLoader = HmdDriverLoader::Instance();

  // RelocalizerThread::RunLocalizer
  HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x1DF010), reinterpret_cast<void *>(RelocalizerThread__RunLocalizerHook),
                       reinterpret_cast<void **>(&RelocalizerThread__RunLocalizer));
}

} // namespace psvr2_toolkit