#pragma once

#include "../hmd_driver_loader.h"

#include <cstdint>

class ConfigManager {
public:
  static constexpr uintptr_t k_createSingletonRVA = 0x1564B0;

  void **vftable;
  uint8_t unk0[0x1568];

  static void createSingleton() {
    if (!ConfigManager__createSingleton) {
      psvr2_toolkit::HmdDriverLoader *pHmdDriverLoader = psvr2_toolkit::HmdDriverLoader::Instance();
      ConfigManager__createSingleton = decltype(ConfigManager__createSingleton)(pHmdDriverLoader->GetBaseAddress() + k_createSingletonRVA);
    }
    ConfigManager__createSingleton();
  }

private:
  inline static void (*ConfigManager__createSingleton)();
};
static_assert(sizeof(ConfigManager) == 0x1570, "Size of ConfigManager is not 0x1570 bytes!");
