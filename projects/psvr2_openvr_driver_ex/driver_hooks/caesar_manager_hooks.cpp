#include "driver_interface/caesar_manager.h"
#include "caesar_manager_hooks.h"

#include "hmd_driver_loader.h"
#include "utils/hook_lib.h"
#include "usb_thread_gaze.h"
#include "utils/driver_settings.h"
#include "util.h"

namespace psvr2_toolkit {
CaesarUsbThreadGaze caesarUsbThreadGaze;

void *(*Framework__Thread__start)(void *thisptr) = nullptr;

void *(*CaesarManager__initialize)(CaesarManager *, void *, void *) = nullptr;
void *CaesarManager__initializeHook(CaesarManager *thisptr, void *arg1, void *arg2) {
  Util::DriverLog("[Gaze] CaesarManager::initialize hook fired.");
  void *result = CaesarManager__initialize(thisptr, arg1, arg2);
  caesarUsbThreadGaze.Start(0);
  Framework__Thread__start(&caesarUsbThreadGaze);

  // Firmware-state reset, restored from the disabled setupManager hook below.
  //
  // That hook was installed at THIS function's address, so PolyHook chained the two: setupManagerHook
  // wrapped this one and cleared these fields after initialize returned. It ran on every initialize,
  // just not where its name suggested. Disabling it in isolation therefore removed a reset that was
  // genuinely happening -- and these two lines are the only place in the toolkit that writes them.
  //
  // Reproduced here in the same order (after the original call) so behaviour is unchanged.
  thisptr->firmwareLoaded = false;
  thisptr->firmwareVersion = 0;

  return result;
}

void (*CaesarManager__shutdown)(void *) = nullptr;
void CaesarManager__shutdownHook(CaesarManager *thisptr) {
  Util::DriverLog("[Gaze] CaesarManager::shutdown hook fired.");
  caesarUsbThreadGaze.JoinThread();

  CaesarManager__shutdown(thisptr);
}

// Currently unreferenced: see the TODO in InstallHooks below. Kept so that re-enabling the hook is a
// one-line change once the correct RVA is known. Note that its body is duplicated into
// CaesarManager__initializeHook above, which is where that work actually used to happen -- drop the
// duplicate there if this hook is ever pointed at the real setupManager.
void *(*CaesarManager__setupManager)(CaesarManager *, void *, void *) = nullptr;
void *CaesarManager__setupManagerHook(CaesarManager *thisptr, void *arg1, void *arg2) {
  Util::DriverLog("[Gaze] CaesarManager::setupManager hook fired.");
  void *result = CaesarManager__setupManager(thisptr, arg1, arg2);

  thisptr->firmwareLoaded = false;
  thisptr->firmwareVersion = 0;

  return result;
}

void CaesarManagerHooks::InstallHooks() {
  static HmdDriverLoader *pHmdDriverLoader = HmdDriverLoader::Instance();

  Framework__Thread__start = decltype(Framework__Thread__start)(pHmdDriverLoader->GetBaseAddress() + 0x16B660);

  if (DriverSettings::IsGazeEnabled()) {
    Util::DriverLog("Enabling PSVR2 gaze tracking...");
    // CaesarManager::initialize
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x123130), reinterpret_cast<void *>(CaesarManager__initializeHook),
                         reinterpret_cast<void **>(&CaesarManager__initialize));

    // CaesarManager::shutdown
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x128320), reinterpret_cast<void *>(CaesarManager__shutdownHook),
                         reinterpret_cast<void **>(&CaesarManager__shutdown));

    // TODO(EYE_TRACKING_PLAN.md 1.1): CaesarManager::setupManager
    //
    // This was installed at 0x123130 -- the same address as CaesarManager::initialize above -- which
    // is a copy-paste of the RVA on the line above it. The effect was that the firmware-state reset in
    // CaesarManager__setupManagerHook never ran where it was meant to, and PolyHook was asked to detour
    // an already-detoured address.
    //
    // The hook is disabled rather than left pointing at the wrong function: an absent hook is honest,
    // a mis-addressed one is not. Re-enable once the real setupManager RVA has been derived from the
    // shipped driver binary.
    //
    // HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x<TBD>),
    //                      reinterpret_cast<void *>(CaesarManager__setupManagerHook),
    //                      reinterpret_cast<void **>(&CaesarManager__setupManager));
  }
}

} // namespace psvr2_toolkit
