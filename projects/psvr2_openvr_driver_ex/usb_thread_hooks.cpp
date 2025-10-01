#include "usb_thread_hooks.h"

#include "hmd_driver_loader.h"
#include "hook_lib.h"
#include "vr_settings.h"
#include "util.h"

#include <thread>
#include <libusb.h>
#include <map>
#include <mutex>

// due to conflicts, have to do this here...
extern "C" {
  typedef PVOID WINUSB_INTERFACE_HANDLE, * PWINUSB_INTERFACE_HANDLE;
  typedef PVOID WINUSB_ISOCH_BUFFER_HANDLE, * PWINUSB_ISOCH_BUFFER_HANDLE;

#pragma pack(1)

  typedef struct _WINUSB_SETUP_PACKET {
    UCHAR   RequestType;
    UCHAR   Request;
    USHORT  Value;
    USHORT  Index;
    USHORT  Length;
  } WINUSB_SETUP_PACKET, * PWINUSB_SETUP_PACKET;

#pragma pack()

  typedef BOOL(WINAPI* WinUsb_Free_t)(WINUSB_INTERFACE_HANDLE InterfaceHandle);
  typedef BOOL(WINAPI* WinUsb_AbortPipe_t)(WINUSB_INTERFACE_HANDLE InterfaceHandle, UCHAR PipeID);
  typedef BOOL(WINAPI* WinUsb_GetCurrentAlternateSetting_t)(WINUSB_INTERFACE_HANDLE InterfaceHandle, PUCHAR SettingNumber);
  typedef BOOL(WINAPI* WinUsb_SetCurrentAlternateSetting_t)(WINUSB_INTERFACE_HANDLE InterfaceHandle, UCHAR SettingNumber);
  typedef BOOL(WINAPI* WinUsb_GetDescriptor_t)(WINUSB_INTERFACE_HANDLE InterfaceHandle, UCHAR DescriptorType, UCHAR Index, USHORT LanguageID, PUCHAR Buffer, ULONG BufferLength, PULONG LengthTransferred);
  typedef BOOL(WINAPI* WinUsb_GetPipePolicy_t)(WINUSB_INTERFACE_HANDLE InterfaceHandle, UCHAR PipeID, ULONG PolicyType, PULONG ValueLength, PVOID Value);
  typedef BOOL(WINAPI* WinUsb_ReadPipe_t)(WINUSB_INTERFACE_HANDLE InterfaceHandle, UCHAR PipeID, PUCHAR Buffer, ULONG BufferLength, PULONG LengthTransferred, LPOVERLAPPED Overlapped);
  typedef BOOL(WINAPI* WinUsb_ControlTransfer_t)(WINUSB_INTERFACE_HANDLE InterfaceHandle, WINUSB_SETUP_PACKET SetupPacket, PUCHAR Buffer, ULONG BufferLength, PULONG LengthTransferred, LPOVERLAPPED Overlapped);

  WinUsb_Free_t o_WinUsb_Free = NULL;
  WinUsb_AbortPipe_t o_WinUsb_AbortPipe = NULL;
  WinUsb_GetCurrentAlternateSetting_t o_WinUsb_GetCurrentAlternateSetting = NULL;
  WinUsb_SetCurrentAlternateSetting_t o_WinUsb_SetCurrentAlternateSetting = NULL;
  WinUsb_GetDescriptor_t o_WinUsb_GetDescriptor = NULL;
  WinUsb_GetPipePolicy_t o_WinUsb_GetPipePolicy = NULL;
  WinUsb_ReadPipe_t o_WinUsb_ReadPipe = NULL;
  WinUsb_ControlTransfer_t o_WinUsb_ControlTransfer = NULL;
}

namespace psvr2_toolkit {
#define PSVR2_VID 0x054c
#define PSVR2_PID 0x0cde

  // === Encapsulated USB Interface State ===
  class LibusbInterface {
  public:
    libusb_device_handle* libusb_handle = NULL;
    int interface_number = -1;
  };

  // === Global State Management ===
  std::mutex g_usb_mutex;
  std::map<WINUSB_INTERFACE_HANDLE, LibusbInterface*> g_interface_map;

  libusb_device_handle* g_handle = NULL; // The single libusb device handle for the PSVR2

  struct Framework__Mutex_t {
    void* __vfptr;
    HANDLE handle;
  };

  struct Framework__Mutex_Vtbl_t {
    Framework__Mutex_t* (*dtor)(Framework__Mutex_t* thisptr);
  };

  enum Framework__Thread__Priority {
    PRIORITY_BELOW_NORMAL = 0,
    PRIORITY_NORMAL = 1,
    PRIORITY_ABOVE_NORMAL = 2,
    PRIORITY_HIGHEST = 3,
    PRIORITY_TIME_CRITICAL = 4
  };

  struct Framework__Thread_t {
    void* __vfptr;
    std::thread* pThread;
    bool inactive;
    uint64_t affinityMask;
    Framework__Thread__Priority priority;
  };

  struct Framework__Thread_Vtbl_t {
    Framework__Thread_t* (*dtor)(Framework__Thread_t* thisptr);
    bool (*task)(Framework__Thread_t* thisptr);
  };

  enum CaesarUsbThread__State {
    STATE_CLOSED = 0,
    STATE_DISCONNECTED = 1,
    STATE_CONNECTED = 2
  };

  struct CaesarUsbThread__Handles_t {
    int initialized;
    void* pInterfaceHandle;
    void* pDeviceHandle;
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
    void (*close)(CaesarUsbThread_t* thisptr);
    void (*__unkFunc3)(CaesarUsbThread_t* thisptr, bool a2);
    char (*getUsbInf)(CaesarUsbThread_t* thisptr);
    char (*getReadPipeId)(CaesarUsbThread_t* thisptr);
    int (*begin)(CaesarUsbThread_t* thisptr);
    void (*__unkFunc7)(CaesarUsbThread_t* thisptr);
    int (*poll)(CaesarUsbThread_t* thisptr);
  };

  void* (*Framework__Mutex__lock)(Framework__Mutex_t* thisptr, uint32_t timeout) = nullptr;
  void* (*Framework__Mutex__unlock)(Framework__Mutex_t* thisptr) = nullptr;

  int (*CaesarUsbThread__read)(void* thisptr, uint8_t pipeId, char* buffer, size_t length) = nullptr;
  int (*CaesarUsbThread__report)(void* thisptr, bool bIsSet, uint16_t reportId, void* buffer, uint16_t length, uint16_t value, uint16_t index, uint16_t subcmd);

  int (*CaesarUsbThreadImuStatus__poll)(void*) = nullptr;
  int CaesarUsbThreadImuStatus__pollHook(void* thisptr) {
    int result = CaesarUsbThreadImuStatus__poll(thisptr);
    CaesarUsbThread__report(thisptr, true, 12, nullptr, 0, 0, 0, 1); // Keep gaze enabled
    return result;
  }

  // We need to replace device enumeration, it is not compatible with LibUSB.
  int CaesarUsbThread__initializeHook(CaesarUsbThread_t* thisptr) {
    // TODO: ShareManager

    std::lock_guard<std::mutex> lock(g_usb_mutex);
    CaesarUsbThread_Vtbl_t* pVtbl = static_cast<CaesarUsbThread_Vtbl_t*>(thisptr->__vfptr);

    Framework__Mutex__lock(&thisptr->handlesMutex, 0xFFFFFFFF);
    thisptr->handles.pDeviceHandle = INVALID_HANDLE_VALUE; // We aren't using this, disables CloseHandle path in Sony code.
    if (thisptr->__unkVar15) {
      Util::DriverLog("[Hook] thisptr->__unkVar15 was true.");
      Framework__Mutex__unlock(&thisptr->handlesMutex);
      return -1;
    }

    byte interface_number = pVtbl->getUsbInf(thisptr);
    Util::DriverLog("[Hook] Driver is requesting to open interface number: {}\n", interface_number);

    // Create our custom interface object to wrap the real handle
    LibusbInterface* new_interface = new LibusbInterface();
    new_interface->interface_number = interface_number;

    // Find the correct vid, pid, and then the correct interface
    static libusb_context* ctx = NULL;
    if (ctx == NULL) {
      if (libusb_init(&ctx) < 0) {
        Util::DriverLog("[libusb hook] Failed to initialize libusb context.\n");
        delete new_interface;
        Framework__Mutex__unlock(&thisptr->handlesMutex);
        return -1;
      }
    }

    if (g_handle == NULL) {
      libusb_device** device_list = NULL;
      ssize_t cnt = libusb_get_device_list(ctx, &device_list);
      if (cnt < 0) {
        Util::DriverLog("[libusb hook] Failed to get device list.\n");
        libusb_exit(ctx);
        delete new_interface;
        Framework__Mutex__unlock(&thisptr->handlesMutex);
        return -1;
      }

      for (ssize_t i = 0; i < cnt; i++) {
        libusb_device* device = device_list[i];
        struct libusb_device_descriptor desc;
        if (libusb_get_device_descriptor(device, &desc) < 0) {
          Framework__Mutex__unlock(&thisptr->handlesMutex);
          return -1;
        }

        // Print all connected USB devices for debugging
        Util::DriverLog("[libusb hook] Found device: VID=0x{}, PID=0x{}\n", desc.idVendor, desc.idProduct);
        // Print all interfaces for this device
        libusb_config_descriptor* config;
        if (libusb_get_config_descriptor(device, 0, &config) < 0) {
          Framework__Mutex__unlock(&thisptr->handlesMutex);
          return -1;
        }
        for (int j = 0; j < config->bNumInterfaces; j++) {
          for (int k = 0; k < config->interface[j].num_altsetting; k++) {
            const libusb_interface_descriptor& iface = config->interface[j].altsetting[k];
            Util::DriverLog("  Interface {}, AltSetting {}, Class 0x%02x, SubClass 0x%02x, Protocol 0x%02x\n",
              iface.bInterfaceNumber, iface.bAlternateSetting, iface.bInterfaceClass, iface.bInterfaceSubClass, iface.bInterfaceProtocol);
          }
        }
        libusb_free_config_descriptor(config);
      }
      libusb_free_device_list(device_list, 1);

      // Open the PSVR2 device if not already opened
      g_handle = libusb_open_device_with_vid_pid(ctx, PSVR2_VID, PSVR2_PID);
      if (g_handle == NULL) {
        Util::DriverLog("[libusb hook] Could not find/open PSVR2 device (VID=0x{}, PID=0x{}).\n", PSVR2_VID, PSVR2_PID);
        delete new_interface;
        Framework__Mutex__unlock(&thisptr->handlesMutex);
        return -1;
      }
      else {
        Util::DriverLog("[libusb hook] Successfully opened PSVR2 device.\n");
      }
    }

    // Claim the requested interface
    if (libusb_claim_interface(g_handle, interface_number) < 0) {
      Util::DriverLog("[libusb hook] Failed to claim interface {}.\n", interface_number);
      libusb_close(g_handle);
      g_handle = NULL;
      delete new_interface;
      Framework__Mutex__unlock(&thisptr->handlesMutex);
      return -1;
    }
    else {
      Util::DriverLog("[libusb hook] Successfully claimed interface {}.\n", interface_number);
    }

    new_interface->libusb_handle = g_handle; // Reuse the single device handle

    // The "fake" handle is a pointer to our struct, which abstracts away the real handle.
    WINUSB_INTERFACE_HANDLE fake_handle = (WINUSB_INTERFACE_HANDLE)new_interface;
    g_interface_map[fake_handle] = new_interface;

    
    thisptr->handles.pInterfaceHandle = fake_handle;
    thisptr->handles.initialized = 1;
    Framework__Mutex__unlock(&thisptr->handlesMutex);

    Util::DriverLog("[Hook] Initialization complete. Fake handle {} created on interface {}.\n", fake_handle, interface_number);
    
    return 0;
  }

  bool WinUsb_AbortPipeHook(WINUSB_INTERFACE_HANDLE InterfaceHandle, UCHAR PipeID) {
    return true; // Return success, even though we're doing nothing. This is missing from LibUSB.
  }

  bool WinUsb_GetCurrentAlternateSettingHook(WINUSB_INTERFACE_HANDLE InterfaceHandle, PUCHAR SettingNumber) {
    LibusbInterface* interface_obj = nullptr;
    {
      std::lock_guard<std::mutex> lock(g_usb_mutex);
      auto it = g_interface_map.find(InterfaceHandle);
      if (it != g_interface_map.end())
        interface_obj = it->second;
    }

    if (interface_obj == nullptr) {
      return o_WinUsb_GetCurrentAlternateSetting(InterfaceHandle, SettingNumber);
    }

    libusb_device* dev = libusb_get_device(interface_obj->libusb_handle);
    libusb_config_descriptor* config;
    if (libusb_get_active_config_descriptor(dev, &config) < 0) {
      Util::DriverLog("[libusb hook] GetCurrentAlternateSetting failed to get config descriptor on interface {}.\n", interface_obj->interface_number);
      return FALSE;
    }
    *SettingNumber = config->interface[0].altsetting[0].bAlternateSetting;
    libusb_free_config_descriptor(config);
    return TRUE;
  }

  bool WinUsb_SetCurrentAlternateSettingHook(WINUSB_INTERFACE_HANDLE InterfaceHandle, UCHAR SettingNumber) {
    LibusbInterface* interface_obj = nullptr;
    {
      std::lock_guard<std::mutex> lock(g_usb_mutex);
      auto it = g_interface_map.find(InterfaceHandle);
      if (it != g_interface_map.end())
        interface_obj = it->second;
    }

    if (interface_obj == nullptr) {
      return o_WinUsb_SetCurrentAlternateSetting(InterfaceHandle, SettingNumber);
    }

    int res = libusb_set_interface_alt_setting(interface_obj->libusb_handle, interface_obj->interface_number, SettingNumber);
    if (res < 0)
    {
      Util::DriverLog("[WinUsb_SetCurrentAlternateSettingHook] SetCurrentAlternateSetting failed on interface {}: %s\n", interface_obj->interface_number, libusb_error_name(res));
      return FALSE;
    }
    return TRUE;
  }

  bool WinUsb_FreeHook(WINUSB_INTERFACE_HANDLE InterfaceHandle) {
    LibusbInterface* interface_obj = nullptr;
    {
      std::lock_guard<std::mutex> lock(g_usb_mutex);
      auto it = g_interface_map.find(InterfaceHandle);
      if (it != g_interface_map.end())
        interface_obj = it->second;
    }

    if (interface_obj == nullptr) {
      Util::DriverLog("[WinUsb_FreeHook] Passing through call for real handle {}.\n", InterfaceHandle);
      return o_WinUsb_Free(InterfaceHandle); // Not our handle, pass it through
    }

    Util::DriverLog("[WinUsb_FreeHook] Intercepted call for fake handle {}.\n", InterfaceHandle);

    delete interface_obj;
    auto it = g_interface_map.find(InterfaceHandle);
    g_interface_map.erase(it);

    if (g_interface_map.empty()) {
      // No more interfaces in use, close the device handle
      if (g_handle) {
        libusb_close(g_handle);
        g_handle = NULL;
        Util::DriverLog("[WinUsb_FreeHook] Closed PSVR2 device handle.\n");
      }
    }

    return TRUE;
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
    LibusbInterface* interface_obj = nullptr;
    {
      std::lock_guard<std::mutex> lock(g_usb_mutex);
      auto it = g_interface_map.find(InterfaceHandle);
      if (it != g_interface_map.end())
        interface_obj = it->second;
    }

    if (interface_obj == nullptr) {
      return o_WinUsb_GetDescriptor(InterfaceHandle, DescriptorType, Index, LanguageID, Buffer, BufferLength, LengthTransferred);
    }

    int res = libusb_get_descriptor(interface_obj->libusb_handle, DescriptorType, Index, Buffer, BufferLength);
    if (res < 0)
    {
      Util::DriverLog("[WinUsb_GetDescriptorHook] GetDescriptor failed on interface {}: %s\n", interface_obj->interface_number, libusb_error_name(res));
      if (LengthTransferred) *LengthTransferred = 0;
      return FALSE;
    }
    if (LengthTransferred) *LengthTransferred = res;
    return TRUE;
  }

  bool WinUsb_GetPipePolicyHook(
    WINUSB_INTERFACE_HANDLE InterfaceHandle,
    UCHAR PipeID,
    ULONG PolicyType,
    PULONG ValueLength,
    PVOID Value
  ) {
    if (PolicyType == 8) {
      *(int*)Value = 1024 * 1024; // hack
    }
    return true; // Also missing from LibUSB.
  }

  bool WinUsb_ReadPipeHook(
    WINUSB_INTERFACE_HANDLE InterfaceHandle,
    UCHAR PipeID,
    PUCHAR Buffer,
    ULONG BufferLength,
    PULONG LengthTransferred,
    LPOVERLAPPED Overlapped
  ) {
    LibusbInterface* interface_obj = nullptr;
    {
      std::lock_guard<std::mutex> lock(g_usb_mutex);
      auto it = g_interface_map.find(InterfaceHandle);
      if (it != g_interface_map.end())
        interface_obj = it->second;
    }

    if (interface_obj == nullptr) {
      return o_WinUsb_ReadPipe(InterfaceHandle, PipeID, Buffer, BufferLength, LengthTransferred, Overlapped);
    }

    int transferred = 0;
    int res = libusb_bulk_transfer(interface_obj->libusb_handle, PipeID, Buffer, BufferLength, &transferred, 0);
    if (res < 0)
    {
      Util::DriverLog("[WinUsb_ReadPipeHook] ReadPipe failed on interface {}, PipeID 0x%02x: %s\n", interface_obj->interface_number, PipeID, libusb_error_name(res));
      if (LengthTransferred) *LengthTransferred = 0;
      return FALSE;
    }
    if (LengthTransferred) *LengthTransferred = transferred;
    return TRUE;
  }

  bool WinUsb_ControlTransferHook(
    WINUSB_INTERFACE_HANDLE* InterfaceHandle,
    WINUSB_SETUP_PACKET SetupPacket,
    PUCHAR Buffer,
    ULONG BufferLength,
    PULONG LengthTransferred,
    LPOVERLAPPED Overlapped
  ) {
    LibusbInterface* interface_obj = nullptr;
    {
      std::lock_guard<std::mutex> lock(g_usb_mutex);
      auto it = g_interface_map.find(InterfaceHandle);
      if (it != g_interface_map.end())
        interface_obj = it->second;
    }

    if (interface_obj == nullptr) {
      return o_WinUsb_ControlTransfer(InterfaceHandle, SetupPacket, Buffer, BufferLength, LengthTransferred, Overlapped);
    }

    int res = libusb_control_transfer(interface_obj->libusb_handle,
      SetupPacket.RequestType,
      SetupPacket.Request,
      SetupPacket.Value,
      SetupPacket.Index,
      Buffer,
      static_cast<uint16_t>(BufferLength),
      5000);

    if (res < 0)
    {
      Util::DriverLog("[WinUsb_ControlTransferHook] ControlTransfer failed on interface {}: %s\n", interface_obj->interface_number, libusb_error_name(res));
      if (LengthTransferred) *LengthTransferred = 0;
      return FALSE;
    }

    *LengthTransferred = res;

    // Don't support Overlapped I/O for now.
    (void)Overlapped; // suppress unused parameter warning

    return TRUE;
  }

  void UsbThreadHooks::InstallHooks() {
    static HmdDriverLoader* pHmdDriverLoader = HmdDriverLoader::Instance();

    Framework__Mutex__lock = decltype(Framework__Mutex__lock)(pHmdDriverLoader->GetBaseAddress() + 0x16B5F0);
    Framework__Mutex__unlock = decltype(Framework__Mutex__unlock)(pHmdDriverLoader->GetBaseAddress() + 0x16B850);

    CaesarUsbThread__read = decltype(CaesarUsbThread__read)(pHmdDriverLoader->GetBaseAddress() + 0x127D60);
    CaesarUsbThread__report = decltype(CaesarUsbThread__report)(pHmdDriverLoader->GetBaseAddress() + 0x1283F0);

    if (!VRSettings::GetBool(STEAMVR_SETTINGS_DISABLE_GAZE, SETTING_DISABLE_GAZE_DEFAULT_VALUE)) {
      // CaesarUsbThreadImuStatus::poll
      HookLib::InstallHook(reinterpret_cast<void*>(pHmdDriverLoader->GetBaseAddress() + 0x1268D0),
                           reinterpret_cast<void*>(CaesarUsbThreadImuStatus__pollHook),
                           reinterpret_cast<void**>(&CaesarUsbThreadImuStatus__poll));
    }

    // LibUSB stuff
    HookLib::InstallHook(reinterpret_cast<void*>(pHmdDriverLoader->GetBaseAddress() + 0x122AC0),
                         reinterpret_cast<void*>(CaesarUsbThread__initializeHook));

    o_WinUsb_AbortPipe = (WinUsb_AbortPipe_t)GetProcAddress(GetModuleHandleW(L"winusb.dll"), "WinUsb_AbortPipe");
    o_WinUsb_GetCurrentAlternateSetting = (WinUsb_GetCurrentAlternateSetting_t)GetProcAddress(GetModuleHandleW(L"winusb.dll"), "WinUsb_GetCurrentAlternateSetting");
    o_WinUsb_SetCurrentAlternateSetting = (WinUsb_SetCurrentAlternateSetting_t)GetProcAddress(GetModuleHandleW(L"winusb.dll"), "WinUsb_SetCurrentAlternateSetting");
    o_WinUsb_Free = (WinUsb_Free_t)GetProcAddress(GetModuleHandleW(L"winusb.dll"), "WinUsb_Free");
    o_WinUsb_GetDescriptor = (WinUsb_GetDescriptor_t)GetProcAddress(GetModuleHandleW(L"winusb.dll"), "WinUsb_GetDescriptor");
    o_WinUsb_GetPipePolicy = (WinUsb_GetPipePolicy_t)GetProcAddress(GetModuleHandleW(L"winusb.dll"), "WinUsb_GetPipePolicy");
    o_WinUsb_ReadPipe = (WinUsb_ReadPipe_t)GetProcAddress(GetModuleHandleW(L"winusb.dll"), "WinUsb_ReadPipe");
    o_WinUsb_ControlTransfer = (WinUsb_ControlTransfer_t)GetProcAddress(GetModuleHandleW(L"winusb.dll"), "WinUsb_ControlTransfer");

    // WinUSB hooks
    HookLib::InstallHook(o_WinUsb_AbortPipe, reinterpret_cast<void*>(WinUsb_AbortPipeHook), (void**)&o_WinUsb_AbortPipe);
    HookLib::InstallHook(o_WinUsb_GetCurrentAlternateSetting, reinterpret_cast<void*>(WinUsb_GetCurrentAlternateSettingHook), (void**)&o_WinUsb_GetCurrentAlternateSetting);
    HookLib::InstallHook(o_WinUsb_SetCurrentAlternateSetting, reinterpret_cast<void*>(WinUsb_SetCurrentAlternateSettingHook), (void**)&o_WinUsb_SetCurrentAlternateSetting);
    HookLib::InstallHook(o_WinUsb_Free, reinterpret_cast<void*>(WinUsb_FreeHook), (void**)&o_WinUsb_Free);
    HookLib::InstallHook(o_WinUsb_GetDescriptor, reinterpret_cast<void*>(WinUsb_GetDescriptorHook), (void**)&o_WinUsb_GetDescriptor);
    HookLib::InstallHook(o_WinUsb_GetPipePolicy, reinterpret_cast<void*>(WinUsb_GetPipePolicyHook), (void**)&o_WinUsb_GetPipePolicy);
    HookLib::InstallHook(o_WinUsb_ReadPipe, reinterpret_cast<void*>(WinUsb_ReadPipeHook), (void**)&o_WinUsb_ReadPipe);
    HookLib::InstallHook(o_WinUsb_ControlTransfer, reinterpret_cast<void*>(WinUsb_ControlTransferHook), (void**)&o_WinUsb_ControlTransfer);
    
  }

} // psvr2_toolkit
