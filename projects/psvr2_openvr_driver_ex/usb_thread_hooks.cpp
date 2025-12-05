#include "usb_thread_hooks.h"

#include "hmd_driver_loader.h"
#include "hook_lib.h"
#include "vr_settings.h"

namespace psvr2_toolkit {
  std::mutex ldPayloadMutex;
  LDPayload currentLDPayload;

  int (*CaesarUsbThread__report)(void *thisptr, bool bIsSet, uint16_t reportId, void *buffer, uint16_t length, uint16_t value, uint16_t index, uint16_t subcmd);

  int (*CaesarUsbThreadImuStatus__poll)(void *) = nullptr;
  int CaesarUsbThreadImuStatus__pollHook(void *thisptr) {
    int result = CaesarUsbThreadImuStatus__poll(thisptr);
    CaesarUsbThread__report(thisptr, true, 12, nullptr, 0, 0, 0, 1); // Keep gaze enabled
    return result;
  }

  uint64_t(*CaesarUsbThreadLeddet__poll)(void* thisptr) = nullptr;
  uint64_t CaesarUsbThreadLeddet__pollHook(void* thisptr) {
    uint64_t result = CaesarUsbThreadLeddet__poll(thisptr);

    std::scoped_lock<std::mutex> lock(ldPayloadMutex);

	  memcpy(&currentLDPayload, reinterpret_cast<uint8_t*>(thisptr) + 0x230, sizeof(LDPayload));

    return result;
  }

  void UsbThreadHooks::InstallHooks() {
    static HmdDriverLoader *pHmdDriverLoader = HmdDriverLoader::Instance();

    CaesarUsbThread__report = decltype(CaesarUsbThread__report)(pHmdDriverLoader->GetBaseAddress() + 0x1283F0);

    if (!VRSettings::GetBool(STEAMVR_SETTINGS_DISABLE_GAZE, SETTING_DISABLE_GAZE_DEFAULT_VALUE)) {
      // CaesarUsbThreadImuStatus::poll
      HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x1268D0),
                           reinterpret_cast<void *>(CaesarUsbThreadImuStatus__pollHook),
                           reinterpret_cast<void **>(&CaesarUsbThreadImuStatus__poll));
    }

    if (VRSettings::GetBool(STEAMVR_SETTINGS_USE_TOOLKIT_SYNC, SETTING_USE_TOOLKIT_SYNC_DEFAULT_VALUE)) {
	    // CaesarUsbThreadLeddet::poll
	    HookLib::InstallHook(reinterpret_cast<void*>(pHmdDriverLoader->GetBaseAddress() + 0x126B80),
		                   reinterpret_cast<void*>(CaesarUsbThreadLeddet__pollHook),
		                   reinterpret_cast<void**>(&CaesarUsbThreadLeddet__poll));
    }
  }

} // psvr2_toolkit
