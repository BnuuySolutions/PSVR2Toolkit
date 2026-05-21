#pragma once

#include "../hmd_driver_loader.h"

#include <cstdint>

// Unscented Kalman Filter
class UkfManager {
public:
  static constexpr uintptr_t k_createSingletonRVA = 0x5D180;

  void **vftable;
  uint8_t unk0[0x4458];

  static void createSingleton(uint8_t a1) {
    if (!UkfManager__createSingleton) {
      psvr2_toolkit::HmdDriverLoader *pHmdDriverLoader = psvr2_toolkit::HmdDriverLoader::Instance();
      UkfManager__createSingleton = decltype(UkfManager__createSingleton)(pHmdDriverLoader->GetBaseAddress() + k_createSingletonRVA);
    }
    UkfManager__createSingleton(a1);
  }

private:
  inline static void (*UkfManager__createSingleton)(uint8_t);
};
static_assert(sizeof(UkfManager) == 0x4460, "Size of UkfManager is not 0x4460 bytes!");
