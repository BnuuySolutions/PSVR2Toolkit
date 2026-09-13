#pragma once

#include "../hmd_driver_loader.h"

#include <cstdint>

#pragma pack(push, 1)

struct CameraConfig {
  float position[3];
  float rotation[9];
  uint8_t pad[0x1C];
};
static_assert(sizeof(CameraConfig) == 0x4C, "Size of CameraConfig is not 0x4C bytes!");

#pragma pack(pop)

class ConfigManager {
public:
  static constexpr uintptr_t k_getSingletonRVA = 0x156470;

  void **vftable;
  uint8_t unk0[0x254];
  CameraConfig cameraConfigs[4];
  uint8_t unk1[0x11E4];

  static ConfigManager *getSingleton() {
    if (!ConfigManager__getSingleton) {
      psvr2_toolkit::HmdDriverLoader *pHmdDriverLoader = psvr2_toolkit::HmdDriverLoader::Instance();
      ConfigManager__getSingleton = decltype(ConfigManager__getSingleton)(pHmdDriverLoader->GetBaseAddress() + k_getSingletonRVA);
    }
    return ConfigManager__getSingleton();
  }

  const CameraConfig *getCameraConfig(uint32_t index) const {
    if (index >= 4) {
      return nullptr;
    }
    return &cameraConfigs[index];
  }

private:
  inline static ConfigManager *(*ConfigManager__getSingleton)();
};
static_assert(sizeof(ConfigManager) == 0x1570, "Size of ConfigManager is not 0x1570 bytes!");
