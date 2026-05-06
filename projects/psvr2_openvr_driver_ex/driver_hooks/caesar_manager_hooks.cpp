#include "driver_interface/caesar_manager.h"
#include "caesar_manager_hooks.h"

#include "hmd_driver_loader.h"
#include "hook_lib.h"
#include "usb_thread_gaze.h"
#include "vr_settings.h"
#include "util.h"

namespace psvr2_toolkit {
  CaesarUsbThreadGaze caesarUsbThreadGaze;

  void *(*Framework__Thread__start)(void *thisptr) = nullptr;

  void *(*CaesarManager__initialize)(CaesarManager *, void *, void *) = nullptr;
  void *CaesarManager__initializeHook(CaesarManager *thisptr, void *arg1, void *arg2) {
    void* result = CaesarManager__initialize(thisptr, arg1, arg2);
    Util::DriverLog("Starting thread");
    caesarUsbThreadGaze.Start(0);
    Framework__Thread__start(&caesarUsbThreadGaze);
    return result;
  }

  void (*CaesarManager__shutdown)(void *) = nullptr;
  void CaesarManager__shutdownHook(CaesarManager *thisptr) {
    caesarUsbThreadGaze.JoinThread();

    CaesarManager__shutdown(thisptr);
  }

  void CaesarManagerHooks::InstallHooks() {
    static HmdDriverLoader *pHmdDriverLoader = HmdDriverLoader::Instance();

    Framework__Thread__start = decltype(Framework__Thread__start)(pHmdDriverLoader->GetBaseAddress() + 0x16B660);

    if (!VRSettings::GetBool(STEAMVR_SETTINGS_DISABLE_GAZE, SETTING_DISABLE_GAZE_DEFAULT_VALUE)) {
      Util::DriverLog("Enabling PSVR2 gaze tracking...");
      // CaesarManager::initialize
      HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x123130),
                           reinterpret_cast<void *>(CaesarManager__initializeHook),
                           reinterpret_cast<void **>(&CaesarManager__initialize));

      // CaesarManager::shutdown
      HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x128320),
                           reinterpret_cast<void *>(CaesarManager__shutdownHook),
                           reinterpret_cast<void **>(&CaesarManager__shutdown));
    }
  }

} // psvr2_toolkit
