#pragma once

#include "aston_context.h"
#include "../hmd_driver_loader.h"

#include <cstdint>

class AstonManager {
public:
  static constexpr uintptr_t k_getSingletonRVA = 0x1189D0;
  static constexpr uintptr_t k_createSingletonRVA = 0x119310;

  void **vftable;
  AstonContext *contexts[2]; // 0 = Right, 1 = Left
  uint8_t unk0[8];

  static AstonManager *getSingleton() {
    if (!AstonManager__getSingleton) {
      psvr2_toolkit::HmdDriverLoader *pHmdDriverLoader = psvr2_toolkit::HmdDriverLoader::Instance();
      AstonManager__getSingleton = decltype(AstonManager__getSingleton)(pHmdDriverLoader->GetBaseAddress() + k_getSingletonRVA);
    }
    return AstonManager__getSingleton();
  }

  static void createSingleton(const char *installPath) {
    if (!AstonManager__createSingleton) {
      psvr2_toolkit::HmdDriverLoader *pHmdDriverLoader = psvr2_toolkit::HmdDriverLoader::Instance();
      AstonManager__createSingleton = decltype(AstonManager__createSingleton)(pHmdDriverLoader->GetBaseAddress() + k_createSingletonRVA);
    }
    AstonManager__createSingleton(installPath);
  }

private:
  inline static AstonManager *(*AstonManager__getSingleton)();
  inline static void (*AstonManager__createSingleton)(const char *);
};
static_assert(sizeof(AstonManager) == 0x20, "Size of AstonManager is not 0x20 bytes!");
