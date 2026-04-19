#include "usb_thread_hooks.h"

#include "hmd_driver_loader.h"
#include "hook_lib.h"
#include "vr_settings.h"
#include "util.h"

#include "custom_share_manager.h"

namespace psvr2_toolkit {
  std::mutex ldPayloadMutex;
  LDPayload currentLDPayload;

  int (*CaesarUsbThread__report)(void *thisptr, bool bIsSet, uint16_t reportId, void *buffer, uint16_t length, uint16_t value, uint16_t index, uint16_t subcmd);

  int (*CaesarUsbThreadImuStatus__poll)(void *) = nullptr;
  int CaesarUsbThreadImuStatus__pollHook(void *thisptr) {
    int result = CaesarUsbThreadImuStatus__poll(thisptr);
    CaesarUsbThread__report(thisptr, true, 12, nullptr, 0, 0, 0, 1); // Keep gaze enabled

    //ipc::IpcServer* pIpcServer = ipc::IpcServer::Instance();
    //if (pIpcServer) {
    //  uint8_t headsetVibration = pIpcServer->HeadsetVibration;
    //  CaesarUsbThread__report(thisptr, true, 8, &headsetVibration, 1, 0, 0, 1);
    //}

    //char data[8] = { 1, 0, 0, 0, 0x05, 0, 0, 0 };

    //CaesarUsbThread__report(thisptr, true, 0xb, data, 8, 0, 0, 1); // Camera Mode
    return result;
  }

  int (*CaesarUsbThreadLeddet__poll)(void *thisptr) = nullptr;
  int CaesarUsbThreadLeddet__pollHook(void *thisptr) {
    int result = CaesarUsbThreadLeddet__poll(thisptr);

    std::scoped_lock<std::mutex> lock(ldPayloadMutex);

    memcpy(&currentLDPayload, reinterpret_cast<uint8_t *>(thisptr) + 0x230, sizeof(LDPayload));

    return result;
  }

  struct image_data {
    unsigned char magic[2];
    uint16_t version;
    uint32_t total_size;
    uint8_t unk0[8];
    uint16_t image_type;
    uint8_t unk1[22];
    uint32_t custom_data_size;
    uint8_t unk2[20];
    uint8_t unk3[192];
    uint8_t __unkData2[2097152];
  };

  // TODO: only partial
  struct CaesarUsbThreadImage {
    unsigned char unk0[0x220];
    image_data image_data;
  };

  int (*CaesarUsbThreadImage__poll)(void *thisptr) = nullptr;
  int CaesarUsbThreadImage__pollHook(void *thisptr) {
    int result = CaesarUsbThreadImage__poll(thisptr);

    if (result == 0) {
      CaesarUsbThreadImage *a1 = (CaesarUsbThreadImage *)thisptr;
      if (a1->image_data.magic[0] == 'V' && a1->image_data.magic[1] == 'I') {
        if (a1->image_data.image_type == 6) {
          CustomShareManager::getSingleton()->setGazeImage((unsigned char *)&a1->image_data);
        }
      }
    }

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
      HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x126B80),
                           reinterpret_cast<void *>(CaesarUsbThreadLeddet__pollHook),
                           reinterpret_cast<void **>(&CaesarUsbThreadLeddet__poll));
    }

    // CaesarUsbThreadImage::poll
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x125E00),
                         reinterpret_cast<void *>(CaesarUsbThreadImage__pollHook),
                         reinterpret_cast<void **>(&CaesarUsbThreadImage__poll));
  }

} // psvr2_toolkit
