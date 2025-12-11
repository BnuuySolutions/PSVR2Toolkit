#include "caesar_manager_hooks.h"

#include "hmd_driver_loader.h"
#include "hook_lib.h"
#include "usb_thread_gaze.h"
#include "vr_settings.h"
#include "util.h"

namespace psvr2_toolkit {

  void *(*Framework__Thread__start)(void *thisptr) = nullptr;

  void *(*CaesarManager__initialize)(void *, void *, void *) = nullptr;
  void *CaesarManager__initializeHook(void *thisptr, void *arg1, void *arg2) {
    static CaesarUsbThreadGaze* pCaesarUsbThreadGaze = CaesarUsbThreadGaze::Instance();

    void* result = CaesarManager__initialize(thisptr, arg1, arg2);
    (*(void (__fastcall **)(__int64, __int64))(*(__int64 *)pCaesarUsbThreadGaze + 24LL))((__int64)pCaesarUsbThreadGaze, 0);
    Framework__Thread__start(pCaesarUsbThreadGaze);
    return result;
  }

  void (*CaesarManager__shutdown)(void *) = nullptr;
  void CaesarManager__shutdownHook(void *thisptr) {
    static CaesarUsbThreadGaze *pCaesarUsbThreadGaze = CaesarUsbThreadGaze::Instance();

    (*(void(__fastcall **)(__int64))(*(__int64 *)pCaesarUsbThreadGaze + 16LL))((__int64)pCaesarUsbThreadGaze);

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
