#include "aston_manager_hooks.h"

#include "hmd_driver_loader.h"
#include "utils/hook_lib.h"
#include "util.h"

#include <cstdint>

namespace psvr2_toolkit {

static constexpr int k_maxPollRateRetries = 8;

static uint32_t (*g_fn_updateControllerPollRate)(int32_t padHandle, uintptr_t device) = nullptr;
static uint32_t UpdateControllerPollRateHook(int32_t padHandle, uintptr_t device) {
  if (device == 0) {
    return g_fn_updateControllerPollRate(padHandle, device);
  }

  uint8_t isLeft = *reinterpret_cast<uint8_t *>(device + 0x3270);
  uint8_t isBt = *reinterpret_cast<uint8_t *>(device + 0x3271);
  uint8_t connectCount = *reinterpret_cast<uint8_t *>(device + 0x1);
  uint64_t lastOutputReportTime = *reinterpret_cast<uint64_t *>(device + 0x2a58);

  int controllerIndex = isLeft ? 0 : 1;
  static uint8_t lastConnectCount[2] = {0, 0};
  static int pollRateRetries[2] = {0, 0};

  if (connectCount != lastConnectCount[controllerIndex] || lastOutputReportTime == 0) {
    lastConnectCount[controllerIndex] = connectCount;
    pollRateRetries[controllerIndex] = 0;
  }

  uint8_t pollRateSet = *reinterpret_cast<uint8_t *>(device + 0x3384);
  int32_t targetPollRateMode = *reinterpret_cast<int32_t *>(device + 0x2a4c);
  uint8_t forceUpdate = *reinterpret_cast<uint8_t *>(device + 0x2a54);

  bool shouldUpdatePollRate =
      isBt != 0 && ((pollRateSet == 0 && targetPollRateMode == 1) || (pollRateSet == 1 && (targetPollRateMode == 0 || forceUpdate != 0)));

  if (!shouldUpdatePollRate) {
    return 0;
  }

  if (pollRateRetries[controllerIndex] >= k_maxPollRateRetries) {
    return 0;
  }

  pollRateRetries[controllerIndex]++;
  Util::DriverLog("[UpdateControllerPollRateHook] Retries: {}/{}", pollRateRetries[controllerIndex], k_maxPollRateRetries);
  return g_fn_updateControllerPollRate(padHandle, device);
}

void AstonManagerHooks::InstallHooks() {
  static HmdDriverLoader *pHmdDriverLoader = HmdDriverLoader::Instance();

  if (Util::IsRunningOnWine()) {
    Util::DriverLog("Limiting controller poll rate update retries due to Linux Bluetooth limitations.");
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x1cdff0), reinterpret_cast<void *>(UpdateControllerPollRateHook),
                         reinterpret_cast<void **>(&g_fn_updateControllerPollRate));
  }

  // AstonManager::getFileFirmwareVersion
  HookLib::InstallStub(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x119400));
}

} // namespace psvr2_toolkit
