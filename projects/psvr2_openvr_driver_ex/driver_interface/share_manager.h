#pragma once

#include "../hmd_driver_loader.h"

#include <cstdint>

class ShareManager {
public:
  static constexpr uintptr_t k_getSingletonRVA = 0x15BBD0;
  static constexpr uintptr_t k_createSingletonRVA = 0x15BCF0;
  static constexpr uintptr_t k_sub_18015E2A0RVA = 0x15E2A0;

  void **vftable;
  uint8_t unk0[0x2428];

  static ShareManager *getSingleton() {
    if (!ShareManager__getSingleton) {
      psvr2_toolkit::HmdDriverLoader *pHmdDriverLoader = psvr2_toolkit::HmdDriverLoader::Instance();
      ShareManager__getSingleton = decltype(ShareManager__getSingleton)(pHmdDriverLoader->GetBaseAddress() + k_getSingletonRVA);
    }
    return ShareManager__getSingleton();
  }

  static void createSingleton(int a1) {
    if (!ShareManager__createSingleton) {
      psvr2_toolkit::HmdDriverLoader *pHmdDriverLoader = psvr2_toolkit::HmdDriverLoader::Instance();
      ShareManager__createSingleton = decltype(ShareManager__createSingleton)(pHmdDriverLoader->GetBaseAddress() + k_createSingletonRVA);
    }
    ShareManager__createSingleton(a1);
  }

  void sub_18015E2A0() {
    if (!ShareManager__sub_18015E2A0) {
      psvr2_toolkit::HmdDriverLoader *pHmdDriverLoader = psvr2_toolkit::HmdDriverLoader::Instance();
      ShareManager__sub_18015E2A0 = decltype(ShareManager__sub_18015E2A0)(pHmdDriverLoader->GetBaseAddress() + k_sub_18015E2A0RVA);
    }
    ShareManager__sub_18015E2A0(this);
  }

private:
  inline static ShareManager *(*ShareManager__getSingleton)();
  inline static void (*ShareManager__createSingleton)(int);
  inline static void (*ShareManager__sub_18015E2A0)(ShareManager *);
};
static_assert(sizeof(ShareManager) == 0x2430, "Size of ShareManager is not 0x2430 bytes!");
