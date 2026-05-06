#include "usb_thread_hooks.h"

#include "hmd_driver_loader.h"
#include "hook_lib.h"
#include "util.h"

#include "custom_share_manager.h"
#include <openvr_driver.h>
#include <filesystem>
#include <fstream>

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

#pragma pack(push, 1)
  struct Hmd2GenDataHeader {
    char magic[2];
    uint16_t minSize;
    uint32_t size;
    uint16_t numItems;
    uint8_t padding[22];
  };

  struct Hmd2GenDataItem {
    uint16_t id;
    uint16_t unk1;
    uint32_t size;
    uint32_t offset;
  };
#pragma pack(pop)

  int customHandleData(char* buffer, uint32_t bufferSize) {
    uint32_t actualBufferSize = bufferSize;

    if (!bufferSize) {
      return 0;
    }

    while (actualBufferSize > 0) {
      if (actualBufferSize < 0x200) {
        return -1;
      }

      Hmd2GenDataHeader* header = reinterpret_cast<Hmd2GenDataHeader*>(buffer);

      if (header->magic[0] != 'V' || header->magic[1] != 'D' ||
          header->size > actualBufferSize || header->minSize != 0x200) {
        return -1;
      }

      if (header->numItems != 0) {
        Hmd2GenDataItem* items = reinterpret_cast<Hmd2GenDataItem*>(buffer + sizeof(Hmd2GenDataHeader));

        for (uint16_t i = 0; i < header->numItems; ++i) {
          Hmd2GenDataItem* item = &items[i];

          if (item->offset + item->size > bufferSize) {
            return -1;
          }

          if (item->id == 0x3 || item->id == 0x4) {
            vr::ETrackedPropertyError err;
            vr::PropertyContainerHandle_t container = vr::VRDriverHandle();
            uint32_t propSize = vr::VRProperties()->GetStringProperty(container, vr::Prop_UserConfigPath_String, nullptr, 0, &err);
            
            if (propSize > 0) {
              std::string configPath(propSize - 1, '\0');
              vr::VRProperties()->GetStringProperty(container, vr::Prop_UserConfigPath_String, configPath.data(), propSize, &err);

              if (err == vr::TrackedProp_Success) {
                std::filesystem::path dir(configPath);
                std::filesystem::path filePath = dir / (std::string("calibration_blob_") + std::to_string(item->id) + ".bin");

                std::ofstream outFile(filePath, std::ios::binary);
                if (outFile.is_open()) {
                  outFile.write(buffer + item->offset, item->size);
                  outFile.close();
                }
              }
            }
          }
        }
      }

      uint32_t size = header->size;
      buffer += size;
      actualBufferSize -= size;
    }

    return 0;
  }

  int (*CaesarUsbThreadGenData__handleData)(void*, char*, uint32_t) = nullptr;
  int CaesarUsbThreadGenData__handleDataHook(void* thisptr, char* buffer, uint32_t bufferSize) {
    int result = CaesarUsbThreadGenData__handleData(thisptr, buffer, bufferSize);
    customHandleData(buffer, bufferSize);
    return result;
  }

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

    // CaesarUsbThreadGenData::handleData
    HookLib::InstallHook(reinterpret_cast<void*>(pHmdDriverLoader->GetBaseAddress() + 0x127250),
                         reinterpret_cast<void*>(CaesarUsbThreadGenData__handleDataHook),
                         reinterpret_cast<void**>(&CaesarUsbThreadGenData__handleData));
  }

} // psvr2_toolkit
