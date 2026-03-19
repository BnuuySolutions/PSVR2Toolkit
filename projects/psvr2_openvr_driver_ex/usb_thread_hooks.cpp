#include "usb_thread_hooks.h"

#include "hmd_driver_loader.h"
#include "hook_lib.h"
#include "vr_settings.h"
#include "ipc_server.h"

namespace psvr2_toolkit {

  int (*CaesarUsbThread__report)(void *thisptr, bool bIsSet, uint16_t reportId, void *buffer, uint16_t length, uint16_t value, uint16_t index, uint16_t subcmd);

  int (*CaesarUsbThreadImuStatus__poll)(void *) = nullptr;
  int CaesarUsbThreadImuStatus__pollHook(void *thisptr) {
    int result = CaesarUsbThreadImuStatus__poll(thisptr);
    CaesarUsbThread__report(thisptr, true, 12, nullptr, 0, 0, 0, 1); // Keep gaze enabled

    ipc::IpcServer* pIpcServer = ipc::IpcServer::Instance();
    if (pIpcServer) {
      uint8_t headsetVibration = pIpcServer->HeadsetVibration;
      CaesarUsbThread__report(thisptr, true, 8, &headsetVibration, 1, 0, 0, 1);
    }

    //char data[8] = { 1, 0, 0, 0, 0x05, 0, 0, 0 };

    //CaesarUsbThread__report(thisptr, true, 0xb, data, 8, 0, 0, 1); // Camera Mode
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
  }

} // psvr2_toolkit
