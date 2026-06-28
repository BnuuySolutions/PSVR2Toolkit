#include "caesar_usb_thread.h"
#include "hmd_driver_loader.h"

#include <cstdint>

using namespace psvr2_toolkit;

#pragma pack(push, 1)

class CaesarManager {
public:
  void** vftable;
  
  uint8_t _pad1[0xB0];
  
  CaesarUsbThread* imuStatusThread;
  CaesarUsbThread* imageThread;
  CaesarUsbThread* slamTrackingThread;
  CaesarUsbThread* leddetThread;
  CaesarUsbThread* genDataThread;
  CaesarUsbThread* relocPreThread;
  CaesarUsbThread* logThread;
  
  static CaesarManager* GetInstance() {
    if (!CaesarManager__getInstance) {
      HmdDriverLoader *pHmdDriverLoader = HmdDriverLoader::Instance();

      CaesarManager__getInstance = decltype(CaesarManager__getInstance)(pHmdDriverLoader->GetBaseAddress() + 0x124c90);
    }

    return CaesarManager__getInstance();
  }

  static void GetIMUTimestampOffset(CaesarManager* caesarManager, int64_t *hmdToHostOffset) {
    if (!CaesarManager__getIMUTimestampOffset) {
      HmdDriverLoader *pHmdDriverLoader = HmdDriverLoader::Instance();

      CaesarManager__getIMUTimestampOffset = decltype(CaesarManager__getIMUTimestampOffset)(pHmdDriverLoader->GetBaseAddress() + 0x1252e0);
    }
    
    CaesarManager__getIMUTimestampOffset(caesarManager, hmdToHostOffset);
  }
private:
  inline static CaesarManager *(*CaesarManager__getInstance)();
  inline static void (*CaesarManager__getIMUTimestampOffset)(CaesarManager *thisptr, int64_t *hmdToHostOffset);
};

#pragma pack(pop)