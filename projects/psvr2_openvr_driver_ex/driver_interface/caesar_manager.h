#pragma once

// TODO: Fix the namespace.
#include "caesar_usb_thread.h"
#include "../hmd_driver_loader.h"

#include <cstdint>

class CaesarManager {
public:
  enum EdidType : uint8_t {
    EDID_TYPE_90HZ_120HZ,
    EDID_TYPE_90HZ_ONLY
  };

  static constexpr uintptr_t k_getSingletonRVA = 0x124C90;
  static constexpr uintptr_t k_getEdidTypeRVA = 0x124970;
  static constexpr uintptr_t k_getUsbBcdRVA = 0x125290;
  static constexpr uintptr_t k_getIMUTimestampOffsetRVA = 0x1252E0;
  static constexpr uintptr_t k_createSingletonRVA = 0x125330;
  static constexpr uintptr_t k_getConnectionStatusRVA = 0x1253C0;
  static constexpr uintptr_t k_setEdidTypeRVA = 0x127E80;

  void **vftable;
  uint8_t unk0[0xB0];
  psvr2_toolkit::CaesarUsbThread *imuStatusThread;
  psvr2_toolkit::CaesarUsbThread *imageThread;
  psvr2_toolkit::CaesarUsbThread *slamTrackingThread;
  psvr2_toolkit::CaesarUsbThread *leddetThread;
  psvr2_toolkit::CaesarUsbThread *genDataThread;
  psvr2_toolkit::CaesarUsbThread *relocPreThread;
  psvr2_toolkit::CaesarUsbThread *logThread;
  uint8_t unk1[0x10];

  static CaesarManager* getSingleton() {
    if (!CaesarManager__getSingleton) {
      psvr2_toolkit::HmdDriverLoader *pHmdDriverLoader = psvr2_toolkit::HmdDriverLoader::Instance();
      CaesarManager__getSingleton = decltype(CaesarManager__getSingleton)(pHmdDriverLoader->GetBaseAddress() + k_getSingletonRVA);
    }
    return CaesarManager__getSingleton();
  }

  int getEdidType(EdidType *out_Type) {
    if (!CaesarManager__getEdidType) {
      psvr2_toolkit::HmdDriverLoader *pHmdDriverLoader = psvr2_toolkit::HmdDriverLoader::Instance();
      CaesarManager__getEdidType = decltype(CaesarManager__getEdidType)(pHmdDriverLoader->GetBaseAddress() + k_getEdidTypeRVA);
    }
    return CaesarManager__getEdidType(this, out_Type);
  }

  int getUsbBcd(uint16_t *out_UsbBcd) {
    if (!CaesarManager__getUsbBcd) {
      psvr2_toolkit::HmdDriverLoader *pHmdDriverLoader = psvr2_toolkit::HmdDriverLoader::Instance();
      CaesarManager__getUsbBcd = decltype(CaesarManager__getUsbBcd)(pHmdDriverLoader->GetBaseAddress() + k_getUsbBcdRVA);
    }
    return CaesarManager__getUsbBcd(this, out_UsbBcd);
  }

  void getIMUTimestampOffset(int64_t *out_HmdToHostOffset) {
    if (!CaesarManager__getIMUTimestampOffset) {
      psvr2_toolkit::HmdDriverLoader *pHmdDriverLoader = psvr2_toolkit::HmdDriverLoader::Instance();
      CaesarManager__getIMUTimestampOffset = decltype(CaesarManager__getIMUTimestampOffset)(pHmdDriverLoader->GetBaseAddress() + k_getIMUTimestampOffsetRVA);
    }
    CaesarManager__getIMUTimestampOffset(this, out_HmdToHostOffset);
  }

  static void createSingleton(const char *installPath, uint8_t a2) {
    if (!CaesarManager__createSingleton) {
      psvr2_toolkit::HmdDriverLoader *pHmdDriverLoader = psvr2_toolkit::HmdDriverLoader::Instance();
      CaesarManager__createSingleton = decltype(CaesarManager__createSingleton)(pHmdDriverLoader->GetBaseAddress() + k_createSingletonRVA);
    }
    CaesarManager__createSingleton(installPath, a2);
  }

  bool getConnectionStatus() {
    if (!CaesarManager__getConnectionStatus) {
      psvr2_toolkit::HmdDriverLoader *pHmdDriverLoader = psvr2_toolkit::HmdDriverLoader::Instance();
      CaesarManager__getConnectionStatus = decltype(CaesarManager__getConnectionStatus)(pHmdDriverLoader->GetBaseAddress() + k_getConnectionStatusRVA);
    }
    return CaesarManager__getConnectionStatus(this);
  }

  int setEdidType(EdidType type) {
    if (!CaesarManager__setEdidType) {
      psvr2_toolkit::HmdDriverLoader *pHmdDriverLoader = psvr2_toolkit::HmdDriverLoader::Instance();
      CaesarManager__setEdidType = decltype(CaesarManager__setEdidType)(pHmdDriverLoader->GetBaseAddress() + k_setEdidTypeRVA);
    }
    return CaesarManager__setEdidType(this, type);
  }

private:
  inline static CaesarManager *(*CaesarManager__getSingleton)();
  inline static int (*CaesarManager__getEdidType)(CaesarManager *, EdidType *);
  inline static int (*CaesarManager__getUsbBcd)(CaesarManager *, uint16_t *);
  inline static void (*CaesarManager__getIMUTimestampOffset)(CaesarManager *, int64_t *);
  inline static void (*CaesarManager__createSingleton)(const char *, uint8_t);
  inline static bool (*CaesarManager__getConnectionStatus)(CaesarManager *);
  inline static int (*CaesarManager__setEdidType)(CaesarManager *, EdidType);
};
static_assert(sizeof(CaesarManager) == 0x100, "Size of CaesarManager is not 0x100 bytes!");
