#include "caesar_manager_hooks.h"

#include "hmd_device_camera.h"
#include "hmd_driver_loader.h"
#include "hook_lib.h"
#include "usb_thread_gaze.h"
#include "vr_settings.h"
#include "util.h"

namespace psvr2_toolkit {

  void *(*Framework__Thread__start)(void *thisptr) = nullptr;

  void* (*CaesarManager__getInstance2)();
  uint64_t(*CaesarManager__getIMUTimestampOffset2)(void* thisptr, int64_t* hmdToHostOffset);

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

  void* (*sie__psvr2__ShareManager_UploadImage)(void*, char*) = nullptr;

  void* sie__psvr2__ShareManager_UploadImageHook(void* thisptr, char* pData) {
      auto res = sie__psvr2__ShareManager_UploadImage(thisptr, pData);

      static HmdDeviceCamera* pHmdDeviceCamera = HmdDeviceCamera::Instance();

      uint32_t hmdTimestamp = *reinterpret_cast<uint32_t*>(pData+8);

      int64_t hmdToHostOffset;

      CaesarManager__getIMUTimestampOffset2(CaesarManager__getInstance2(), &hmdToHostOffset);

      double timeOffset = (static_cast<int64_t>(hmdTimestamp) + hmdToHostOffset) / 1e6;

      static LARGE_INTEGER frequency{};
      if (frequency.QuadPart == 0)
      {
          QueryPerformanceFrequency(&frequency);
      }

      uint64_t ticks = static_cast<uint64_t>(timeOffset * static_cast<double>(frequency.QuadPart));

      pHmdDeviceCamera->UploadBC4(ticks, reinterpret_cast<uint8_t*>(pData + 256));

      return res;
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
    void* psie__psvr2__ShareManager_UploadImage = reinterpret_cast<void*>(pHmdDriverLoader->GetBaseAddress() + 0x128030);
    HookLib::InstallHook(psie__psvr2__ShareManager_UploadImage,
        reinterpret_cast<void*>(sie__psvr2__ShareManager_UploadImageHook),
        reinterpret_cast<void**>(&sie__psvr2__ShareManager_UploadImage));

    CaesarManager__getInstance2 = decltype(CaesarManager__getInstance2)(pHmdDriverLoader->GetBaseAddress() + 0x124c90);
    CaesarManager__getIMUTimestampOffset2 = decltype(CaesarManager__getIMUTimestampOffset2)(pHmdDriverLoader->GetBaseAddress() + 0x1252e0);
  }

} // psvr2_toolkit
