#include "usb_thread_hooks.h"

#include "hmd_driver_loader.h"
#include "hook_lib.h"
#include "vr_settings.h"

#include <thread>

namespace psvr2_toolkit {

  struct Framework__Mutex_t {
    void *__vfptr;
    HANDLE handle;
  };

  struct Framework__Mutex_Vtbl_t {
    Framework__Mutex_t *(*dtor)(Framework__Mutex_t *thisptr);
  };

  enum Framework__Thread__Priority {
    PRIORITY_BELOW_NORMAL = 0,
    PRIORITY_NORMAL = 1,
    PRIORITY_ABOVE_NORMAL = 2,
    PRIORITY_HIGHEST = 3,
    PRIORITY_TIME_CRITICAL = 4
  };

  struct Framework__Thread_t {
    void *__vfptr;
    std::thread *pThread;
    bool inactive;
    uint64_t affinityMask;
    Framework__Thread__Priority priority;
  };

  struct Framework__Thread_Vtbl_t {
    Framework__Thread_t *(*dtor)(Framework__Thread_t *thisptr);
    bool (*task)(Framework__Thread_t *thisptr);
  };

  enum CaesarUsbThread__State {
    STATE_CLOSED = 0,
    STATE_DISCONNECTED = 1,
    STATE_CONNECTED = 2
  };

  struct CaesarUsbThread__Handles_t {
    int initialized;
    void *pInterfaceHandle;
    void *pDeviceHandle;
  };

  struct CaesarUsbThread_t {
    Framework__Thread_t __base_Framework__Thread;
    CaesarUsbThread__State state;
    char __unkData6[4];
    Framework__Mutex_t mutex;
    CaesarUsbThread__Handles_t handles;
    char __unkData12[260];
    char driverInfo[128];
    char __unkData14[4];
    bool __unkVar15;
    char __unkData16[3];
    uint32_t counter;
    uint32_t __unkVar18;
    char __unkData19[44];
    uint32_t lastError;
  };

  int (*CaesarUsbThread__report)(void *thisptr, bool bIsSet, uint16_t reportId, void *buffer, uint16_t length, uint16_t value, uint16_t index, uint16_t subcmd);

  int (*CaesarUsbThreadImuStatus__poll)(void *) = nullptr;
  int CaesarUsbThreadImuStatus__pollHook(void *thisptr) {
    int result = CaesarUsbThreadImuStatus__poll(thisptr);
    CaesarUsbThread__report(thisptr, true, 12, nullptr, 0, 0, 0, 1); // Keep gaze enabled
    return result;
  }

  void CaesarUsbThread__freeHook(CaesarUsbThread_t *thisptr, CaesarUsbThread__Handles_t *handles) {
    if (handles->initialized) {
      // TODO: Free LibUSB interface handle.
      // We do not need to free the device (file) handle.
      handles->initialized = 0;
    }
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

    // LibUSB stuff
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x122A70),
                         reinterpret_cast<void *>(CaesarUsbThread__freeHook));
  }

} // psvr2_toolkit
