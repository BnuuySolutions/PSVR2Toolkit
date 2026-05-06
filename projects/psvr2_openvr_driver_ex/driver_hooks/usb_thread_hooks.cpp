#include "usb_thread_hooks.h"

#include "hmd_driver_loader.h"
#include "hook_lib.h"

#include "custom_share_manager.h"

namespace psvr2_toolkit {
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

    // CaesarUsbThreadImage::poll
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x125E00),
                         reinterpret_cast<void *>(CaesarUsbThreadImage__pollHook),
                         reinterpret_cast<void **>(&CaesarUsbThreadImage__poll));
  }

} // psvr2_toolkit
