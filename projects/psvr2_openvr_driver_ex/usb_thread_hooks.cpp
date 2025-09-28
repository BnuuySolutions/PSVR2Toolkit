#include "usb_thread_hooks.h"

#include "hmd_driver_loader.h"
#include "hook_lib.h"
#include "vr_settings.h"
#include "util.h"

#include <thread>
#include <lusb0_usb.h>

// due to conflicts, have to do this here...
extern "C" {
  typedef PVOID WINUSB_INTERFACE_HANDLE, *PWINUSB_INTERFACE_HANDLE;
  typedef PVOID WINUSB_ISOCH_BUFFER_HANDLE, *PWINUSB_ISOCH_BUFFER_HANDLE;

#pragma pack(1)

  typedef struct _WINUSB_SETUP_PACKET {
    UCHAR   RequestType;
    UCHAR   Request;
    USHORT  Value;
    USHORT  Index;
    USHORT  Length;
  } WINUSB_SETUP_PACKET, *PWINUSB_SETUP_PACKET;

#pragma pack()

  BOOL __stdcall WinUsb_Initialize(HANDLE DeviceHandle, PWINUSB_INTERFACE_HANDLE InterfaceHandle);
  BOOL __stdcall WinUsb_Free(WINUSB_INTERFACE_HANDLE InterfaceHandle);
  BOOL __stdcall WinUsb_AbortPipe(WINUSB_INTERFACE_HANDLE InterfaceHandle, UCHAR PipeID);
  BOOL __stdcall WinUsb_GetCurrentAlternateSetting(WINUSB_INTERFACE_HANDLE InterfaceHandle, PUCHAR SettingNumber);
  BOOL __stdcall WinUsb_SetCurrentAlternateSetting(WINUSB_INTERFACE_HANDLE InterfaceHandle, UCHAR SettingNumber);
  BOOL __stdcall WinUsb_GetDescriptor(WINUSB_INTERFACE_HANDLE InterfaceHandle, UCHAR DescriptorType, UCHAR Index, USHORT LanguageID, PUCHAR Buffer, ULONG BufferLength, PULONG LengthTransferred);
  BOOL __stdcall WinUsb_GetPipePolicy(WINUSB_INTERFACE_HANDLE InterfaceHandle, UCHAR PipeID, ULONG PolicyType, PULONG ValueLength, PVOID Value);
  BOOL __stdcall WinUsb_ReadPipe(WINUSB_INTERFACE_HANDLE InterfaceHandle, UCHAR PipeID, PUCHAR Buffer, ULONG BufferLength, PULONG LengthTransferred, LPOVERLAPPED Overlapped);
  BOOL __stdcall WinUsb_ControlTransfer(WINUSB_INTERFACE_HANDLE InterfaceHandle, WINUSB_SETUP_PACKET SetupPacket, PUCHAR Buffer, ULONG BufferLength, PULONG LengthTransferred, LPOVERLAPPED Overlapped);
}

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

  struct CaesarUsbThread_t : Framework__Thread_t {
    CaesarUsbThread__State state;
    char __unkData6[4];
    Framework__Mutex_t handlesMutex;
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

  struct CaesarUsbThread_Vtbl_t : Framework__Thread_Vtbl_t {
    void (*close)(CaesarUsbThread_t *thisptr);
    void (*__unkFunc3)(CaesarUsbThread_t *thisptr, bool a2);
    char (*getUsbInf)(CaesarUsbThread_t *thisptr);
    char (*getReadPipeId)(CaesarUsbThread_t *thisptr);
    int (*begin)(CaesarUsbThread_t *thisptr);
    void (*__unkFunc7)(CaesarUsbThread_t *thisptr);
    int (*poll)(CaesarUsbThread_t *thisptr);
  };

  void *(*Framework__Mutex__lock)(Framework__Mutex_t *thisptr, uint32_t timeout) = nullptr;
  void *(*Framework__Mutex__unlock)(Framework__Mutex_t *thisptr) = nullptr;

  int (*CaesarUsbThread__read)(void *thisptr, uint8_t pipeId, char *buffer, size_t length) = nullptr;
  int (*CaesarUsbThread__report)(void *thisptr, bool bIsSet, uint16_t reportId, void *buffer, uint16_t length, uint16_t value, uint16_t index, uint16_t subcmd);

  int (*CaesarUsbThreadImuStatus__poll)(void *) = nullptr;
  int CaesarUsbThreadImuStatus__pollHook(void *thisptr) {
    int result = CaesarUsbThreadImuStatus__poll(thisptr);
    CaesarUsbThread__report(thisptr, true, 12, nullptr, 0, 0, 0, 1); // Keep gaze enabled
    return result;
  }

  struct Init {
    struct usb_device *dev;
    char usbInf;
  };

  // We need to replace device enumeration, it is not compatible with LibUSB.
  int CaesarUsbThread__initializeHook(CaesarUsbThread_t *thisptr) {
    // TODO: ShareManager
    CaesarUsbThread_Vtbl_t *pVtbl = static_cast<CaesarUsbThread_Vtbl_t *>(thisptr->__vfptr);
    char usbInf = pVtbl->getUsbInf(thisptr);
    struct usb_device *actual_dev = nullptr;
    Util::DriverLog("Before Get Busses");
    for (struct usb_bus *bus = usb_get_busses(); bus; bus = bus->next) {
      for (struct usb_device *dev = bus->devices; dev; dev = dev->next) {
        for (int i = 0; i < dev->config->bNumInterfaces; i++) {
          struct usb_interface *intf = &dev->config->interface[i];
          for (int j = 0; j < intf->num_altsetting; j++) {
            struct usb_interface_descriptor *desc = &intf->altsetting[j];
            Util::DriverLog("idVendor = {}, idProduct = {}, bInterfaceNumber = {}",
              dev->descriptor.idVendor,
              dev->descriptor.idProduct,
              desc->bInterfaceNumber);

            if (dev->descriptor.idVendor == 0x054C &&
              dev->descriptor.idProduct == 0x0CDE &&
              desc->bInterfaceNumber == usbInf) {
              Util::DriverLog("Found device");
              Util::DriverLog("dev = {}", (uintptr_t)dev);
              actual_dev = dev;
              break;
            }
          }
        }
      }
    }
    Util::DriverLog("Locking mutex");
    Framework__Mutex__lock(&thisptr->handlesMutex, 0xFFFFFFFF);
    thisptr->handles.pDeviceHandle = INVALID_HANDLE_VALUE; // We aren't using this, disables CloseHandle path in Sony code.
    if (thisptr->__unkVar15) {
      Util::DriverLog("Unlocking mutex");
      Framework__Mutex__unlock(&thisptr->handlesMutex);
      return -1;
    }
    Util::DriverLog("Before winusb initialize (HOOKED)");
    Init init;
    init.dev = actual_dev;
    init.usbInf = usbInf;
    if (!WinUsb_Initialize(&init, &thisptr->handles.pInterfaceHandle)) {
      Util::DriverLog("Unlocking mutex");
      Framework__Mutex__unlock(&thisptr->handlesMutex);
      return -1;
    }
    Util::DriverLog("Unlocking mutex");
    Framework__Mutex__unlock(&thisptr->handlesMutex);
    // TODO: Driver info
    Util::DriverLog("handles init");
    thisptr->handles.initialized = 1;
    return 0;
  }

  bool WinUsb_InitializeHook(HANDLE DeviceHandle, PWINUSB_INTERFACE_HANDLE InterfaceHandle) {
    Init *init = (Init *)DeviceHandle;
    Util::DriverLog("WinUsb_InitializeHook call... dev = {}, usbInf = {}", init->dev == nullptr ? "null" : "not null", (int)init->usbInf);
    Util::DriverLog("WinUsb_InitializeHook dev = {}", (uintptr_t)init->dev);
    Util::DriverLog("WinUsb_InitializeHook dev cast = {}", (uintptr_t)static_cast<struct usb_device *>(init->dev));
    usb_dev_handle *handle = usb_open(static_cast<struct usb_device *>(init->dev));
    if (usb_claim_interface(handle, init->usbInf) != 0) {
      Util::DriverLog("Failed to claim interface {}", init->usbInf);
      usb_close(handle);
      *InterfaceHandle = INVALID_HANDLE_VALUE;
      return false;
    }
    Util::DriverLog("Usb open success");
    *InterfaceHandle = handle;
    return true;
  }

  bool WinUsb_AbortPipeHook(WINUSB_INTERFACE_HANDLE InterfaceHandle, UCHAR PipeID) {
    return true; // Return success, even though we're doing nothing. This is missing from LibUSB0.
  }

  bool WinUsb_GetCurrentAlternateSettingHook(WINUSB_INTERFACE_HANDLE InterfaceHandle, PUCHAR SettingNumber) {
    *SettingNumber = 1;
    return true; // Also missing from LibUSB0.
  }

  bool WinUsb_SetCurrentAlternateSettingHook(WINUSB_INTERFACE_HANDLE InterfaceHandle, UCHAR SettingNumber) {
    return true; // Also missing from LibUSB0.
  }

  bool WinUsb_FreeHook(WINUSB_INTERFACE_HANDLE InterfaceHandle) {
    if (InterfaceHandle != INVALID_HANDLE_VALUE) {
      usb_close(static_cast<struct usb_dev_handle *>(InterfaceHandle));
      return true;
    }
    return false;
  }

  bool WinUsb_GetDescriptorHook(
    WINUSB_INTERFACE_HANDLE InterfaceHandle,
    UCHAR DescriptorType,
    UCHAR Index,
    USHORT LanguageID,
    PUCHAR Buffer,
    ULONG BufferLength,
    PULONG LengthTransferred
  ) {
    if (InterfaceHandle == INVALID_HANDLE_VALUE || !Buffer) {
      return false;
    }

    int ret = usb_control_msg(
      static_cast<struct usb_dev_handle *>(InterfaceHandle),
      USB_ENDPOINT_IN,
      USB_REQ_GET_DESCRIPTOR,
      (DescriptorType << 8) | Index,
      LanguageID,
      (char *)Buffer,
      (int)BufferLength,
      1000
    );

    if (ret < 0) {
      if (LengthTransferred) {
        *LengthTransferred = 0;
      }
      return false;
    }

    if (LengthTransferred) {
      *LengthTransferred = ret;
    }

    return true;
  }

  bool WinUsb_GetPipePolicyHook(
    WINUSB_INTERFACE_HANDLE InterfaceHandle,
    UCHAR PipeID,
    ULONG PolicyType,
    PULONG ValueLength,
    PVOID Value
  ) {
    if (PolicyType == 8) {
      *(int *)Value = 1024 * 1024; // hack
    }
    return true; // Also missing from LibUSB0.
  }

  bool WinUsb_ReadPipeHook(
    WINUSB_INTERFACE_HANDLE InterfaceHandle,
    UCHAR PipeID,
    PUCHAR Buffer,
    ULONG BufferLength,
    PULONG LengthTransferred,
    LPOVERLAPPED Overlapped
  ) {
    if (InterfaceHandle == INVALID_HANDLE_VALUE || !Buffer) {
      return false;
    }

    int transferred = 0;
    int timeout = 1000; // ms, adjust to your needs

    // Choose bulk or interrupt depending on endpoint attributes
    if ((PipeID & USB_ENDPOINT_TYPE_MASK) == USB_ENDPOINT_TYPE_INTERRUPT) {
      transferred = usb_interrupt_read(
        static_cast<struct usb_dev_handle *>(InterfaceHandle),
        PipeID,
        (char *)Buffer,
        (int)BufferLength,
        timeout
      );
    }
    else {
      transferred = usb_bulk_read(
        static_cast<struct usb_dev_handle *>(InterfaceHandle),
        PipeID,
        (char *)Buffer,
        (int)BufferLength,
        timeout
      );
    }

    if (transferred < 0) {
      if (LengthTransferred) {
        *LengthTransferred = 0;
      }
      return false;
    }

    if (LengthTransferred) {
      *LengthTransferred = transferred;
    }

    return true;
  }

  bool WinUsb_ControlTransferHook(
    usb_dev_handle *InterfaceHandle,
    WINUSB_SETUP_PACKET SetupPacket,
    PUCHAR Buffer,
    ULONG BufferLength,
    PULONG LengthTransferred,
    LPOVERLAPPED Overlapped
  ) {
    if (!InterfaceHandle) {
      return FALSE;
    }

    int timeout = 1000; // ms, adjust as needed

    int ret = usb_control_msg(
      InterfaceHandle,
      SetupPacket.RequestType,
      SetupPacket.Request,
      SetupPacket.Value,
      SetupPacket.Index,
      (char *)Buffer,
      (int)BufferLength,
      timeout
    );

    if (ret < 0) {
      if (LengthTransferred) {
        *LengthTransferred = 0;
      }
      return FALSE;
    }

    if (LengthTransferred) {
      *LengthTransferred = (ULONG)ret;
    }

    // libusb-0.1 does not support Overlapped I/O
    (void)Overlapped; // suppress unused parameter warning

    return TRUE;
  }

  void UsbThreadHooks::InstallHooks() {
    static HmdDriverLoader *pHmdDriverLoader = HmdDriverLoader::Instance();

    Framework__Mutex__lock = decltype(Framework__Mutex__lock)(pHmdDriverLoader->GetBaseAddress() + 0x16B5F0);
    Framework__Mutex__unlock = decltype(Framework__Mutex__unlock)(pHmdDriverLoader->GetBaseAddress() + 0x16B850);

    CaesarUsbThread__read = decltype(CaesarUsbThread__read)(pHmdDriverLoader->GetBaseAddress() + 0x127D60);
    CaesarUsbThread__report = decltype(CaesarUsbThread__report)(pHmdDriverLoader->GetBaseAddress() + 0x1283F0);

    if (!VRSettings::GetBool(STEAMVR_SETTINGS_DISABLE_GAZE, SETTING_DISABLE_GAZE_DEFAULT_VALUE)) {
      // CaesarUsbThreadImuStatus::poll
      HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x1268D0),
                           reinterpret_cast<void *>(CaesarUsbThreadImuStatus__pollHook),
                           reinterpret_cast<void **>(&CaesarUsbThreadImuStatus__poll));
    }

    // TODO FIX!!
    Util::DriverLog("Before Init");
    usb_init(); /* initialize the library */
    Util::DriverLog("Before Find Busses");
    usb_find_busses(); /* find all busses */
    Util::DriverLog("Before Find Devices");
    usb_find_devices(); /* find all connected devices */

    // LibUSB stuff
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x122AC0),
                         reinterpret_cast<void *>(CaesarUsbThread__initializeHook));

    // WinUSB hooks
    HookLib::InstallHook(&WinUsb_Initialize, reinterpret_cast<void *>(WinUsb_InitializeHook));
    HookLib::InstallHook(&WinUsb_AbortPipe, reinterpret_cast<void *>(WinUsb_AbortPipeHook));
    HookLib::InstallHook(&WinUsb_GetCurrentAlternateSetting, reinterpret_cast<void *>(WinUsb_GetCurrentAlternateSettingHook));
    HookLib::InstallHook(&WinUsb_SetCurrentAlternateSetting, reinterpret_cast<void *>(WinUsb_SetCurrentAlternateSettingHook));
    HookLib::InstallHook(&WinUsb_Free, reinterpret_cast<void *>(WinUsb_FreeHook));
    HookLib::InstallHook(&WinUsb_GetDescriptor, reinterpret_cast<void *>(WinUsb_GetDescriptorHook));
    HookLib::InstallHook(&WinUsb_GetPipePolicy, reinterpret_cast<void *>(WinUsb_GetPipePolicyHook));
    HookLib::InstallHook(&WinUsb_ReadPipe, reinterpret_cast<void *>(WinUsb_ReadPipeHook));
    HookLib::InstallHook(&WinUsb_ControlTransfer, reinterpret_cast<void *>(WinUsb_ControlTransferHook));
  }

} // psvr2_toolkit
