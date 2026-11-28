#include "config_manager_hooks.h"

#include "hmd_driver_loader.h"
#include "util.h"
#include "utils/hook_lib.h"

#include <cstdint>
#include <cstring>

namespace psvr2_toolkit {

static void (*ConfigManager__UpdateTransforms)(void *pConfigManager) = nullptr;
static void ConfigManager__UpdateTransformsHook(void *pConfigManager) {
  ConfigManager__UpdateTransforms(pConfigManager);

  // Sony's IR tracker loop uses the uncalibrated default IMU-to-camera poses at ConfigManager+0x4DC,
  // however, the calibrated poses exist at +0x48C. There's also the inverse at +0x504
  // that seems unused, but we'll replace it regardless just to be safe...
  if (pConfigManager) {
    auto *cm = reinterpret_cast<uint8_t *>(pConfigManager);
    std::memcpy(cm + 0x4DC, cm + 0x48C, 0x28);
    std::memcpy(cm + 0x504, cm + 0x4B4, 0x28);
    Util::DriverLog("[ConfigManager__UpdateTransformsHook] Replaced uncalibrated poses with calibrated versions");
  }
}

void ConfigManagerHooks::InstallHooks() {
  static HmdDriverLoader *pHmdDriverLoader = HmdDriverLoader::Instance();
  uintptr_t baseAddress = pHmdDriverLoader->GetBaseAddress();

  HookLib::InstallHook(reinterpret_cast<void *>(baseAddress + 0x1556B0), reinterpret_cast<void *>(ConfigManager__UpdateTransformsHook),
                       reinterpret_cast<void **>(&ConfigManager__UpdateTransforms));
}

} // namespace psvr2_toolkit
