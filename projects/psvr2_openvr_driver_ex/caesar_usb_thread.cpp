#include "caesar_usb_thread.h"
#include "hmd_driver_loader.h"
#include "hook_lib.h"
#include "libusb-1.0/libusb.h"
#include "util.h"

#include <debugapi.h>
#include <mutex>
#include <shared_mutex>

#define IS_HANDLE_VALID(handle) (reinterpret_cast<uint64_t>(handle) != -1) 

namespace psvr2_toolkit {

  static libusb_context* g_usbCtx = INVALID_CONTEXT_HANDLE;
  static libusb_device_handle* g_devHandle = INVALID_DEVICE_HANDLE;
  static std::mutex g_devHandleMutex;
  static bool g_usbSimulatedDisconnect = false;

  static std::shared_mutex g_pendingOperationsMutex;

  void (*CaesarUsbThread::orig_destructor)(CaesarUsbThread* thisptr, bool shouldFree) = nullptr;
  void (*CaesarUsbThread::orig_threadLoop)(CaesarUsbThread* thisptr) = nullptr;
  void (*CaesarUsbThread::orig_joinThread)(CaesarUsbThread* thisptr) = nullptr;
  void (*CaesarUsbThread::orig_start)(CaesarUsbThread* thisptr, bool connectNow) = nullptr;
  uint8_t (*CaesarUsbThread::orig_getInterface)(CaesarUsbThread* thisptr) = nullptr;
  uint8_t (*CaesarUsbThread::orig_getEndpoint)(CaesarUsbThread* thisptr) = nullptr;
  void (*CaesarUsbThread::orig_onConnected)(CaesarUsbThread* thisptr) = nullptr;
  void (*CaesarUsbThread::orig_onDisconnect)(CaesarUsbThread* thisptr) = nullptr;
  int (*CaesarUsbThread::orig_pollAndProcess)(CaesarUsbThread* thisptr) = nullptr;

  int (*CaesarUsbThread::orig_read)(CaesarUsbThread* thisptr, uint8_t pipeId, char* buffer, size_t length) = nullptr;
  int (*CaesarUsbThread::orig_report)(CaesarUsbThread* thisptr, uint8_t bIsSet, uint16_t reportId, void* buffer, uint16_t length, uint16_t value, uint16_t index, uint16_t subcmd) = nullptr;
  bool (*CaesarUsbThread::orig_getDescriptor)(CaesarUsbThread* thisptr, libusb_device_descriptor* pDest) = nullptr;

  CaesarUsbThread::~CaesarUsbThread() {
    // We don't free because specific destructor is for objects we made.
    DestructorHook(this, false);
  }
  void CaesarUsbThread::ThreadLoop() { ThreadLoopHook(this); }
  void CaesarUsbThread::JoinThread() { JoinThreadHook(this); }
  void CaesarUsbThread::Start(bool connectNow) { StartHook(this, connectNow); }

  // Base stub implementations (These are overridden by the implementations)
  uint8_t CaesarUsbThread::GetInterface() { return 0; }
  uint8_t CaesarUsbThread::GetEndpoint() { return 0; }
  void CaesarUsbThread::OnConnected() {}
  void CaesarUsbThread::OnDisconnect() {}
  int CaesarUsbThread::PollAndProcess() { return 0; }

  void CaesarUsbThread::SetUsbConnectionState(bool connected) {
    std::lock_guard<std::mutex> lock(g_devHandleMutex);
    g_usbSimulatedDisconnect = !connected;
    if (g_usbSimulatedDisconnect && IS_HANDLE_VALID(g_devHandle)) {
      // Wait for all pending operations to finish with an exclusive lock
      std::unique_lock<std::shared_mutex> opsLock(g_pendingOperationsMutex);
      libusb_close(g_devHandle);
      g_devHandle = INVALID_DEVICE_HANDLE;
    }
  }

  int CaesarUsbThread::TransferPipe(uint8_t pipeId, char* buffer, size_t length) {
    std::shared_lock<std::shared_mutex> lock(g_pendingOperationsMutex);
    if (this->m_stopRequested != 0 || g_usbSimulatedDisconnect) {
      this->m_lastError = 0xfffffe74;
      return -1;
    }
    if (!IS_HANDLE_VALID(this->m_devHandle)) {
      this->m_lastError = 0xfffffe79;
      return -1;
    }

    bool isBulk = this->GetInterface() != 7;

    int transferred = 0;
    int result = 0;

    if (isBulk) {
      result = libusb_bulk_transfer(
        this->m_devHandle,
        pipeId,
        (unsigned char*)buffer,
        static_cast<int>(length),
        &transferred,
        1000);
    } else {
      result = libusb_interrupt_transfer(
        this->m_devHandle,
        pipeId,
        (unsigned char*)buffer,
        static_cast<int>(length),
        &transferred,
        1000);
    }

    if (result != 0) {
      if (result == LIBUSB_ERROR_TIMEOUT) {
        return 0;
      }
      this->m_lastError = result;
      return -1; // Only return -1 for actual fatal USB errors
    }

    return transferred;
  }

  int CaesarUsbThread::ControlCommand(uint8_t bIsSet, uint16_t reportId, void* buffer, uint16_t length, uint16_t value, uint16_t index, uint16_t& subcmd) {
    std::shared_lock<std::shared_mutex> lock(g_pendingOperationsMutex);
    if (this->m_stopRequested != 0 || g_usbSimulatedDisconnect) {
      this->m_lastError = 0xfffffdd4;
      Util::DriverLog("Stop requested. Report ID: {}", reportId);
      return -1;
    }
    if (!IS_HANDLE_VALID(this->m_devHandle)) {
      this->m_lastError = 0xfffffdcd;
      Util::DriverLog("No dev {}. Report ID: {}", (uint64_t)this, reportId);
      return -1;
    }

    uint8_t bmRequestType = bIsSet == 0 ? 0xC2 : 0x42;
    uint8_t bRequest = bIsSet == 0 ? 0x01 : 0x09;
    uint16_t wValue = bIsSet == 0 ? reportId : value;
    uint16_t wIndex = index;
    uint16_t wLength = length + 8;

#pragma pack(push, 1)
    struct ControlPayload {
      uint8_t reportId;
      uint8_t zero;
      uint16_t subcmd;
      uint16_t length;
      uint16_t zero2;
      uint8_t data[504];
    };
#pragma pack(pop)

    ControlPayload payload;
    memset(&payload, 0, sizeof(payload));
    payload.reportId = static_cast<uint8_t>(reportId);
    payload.subcmd = subcmd;
    payload.length = length;

    if (bIsSet != 0 && buffer && length > 0) {
      memcpy(payload.data, buffer, length);
    }

    int result = libusb_control_transfer(
      this->m_devHandle,
      bmRequestType,
      bRequest,
      wValue,
      wIndex,
      reinterpret_cast<unsigned char*>(&payload),
      wLength,
      1000);

    if (result < 0) {
      Util::DriverLog("[Error] {} Report failed. (libusb_error={})", bIsSet != 0 ? "Set" : "Get", result);
      this->m_lastError = result;
      return -1;
    } else if (bIsSet == 0 && buffer && length > 0) {
      subcmd = payload.subcmd; // Doubles as a result, mostly for gaze.
      memcpy(buffer, payload.data, length);
    }
    return result;
  }

  int CaesarUsbThread::GetDescriptor(libusb_device_descriptor* pDest) {
    std::shared_lock<std::shared_mutex> lock(g_pendingOperationsMutex);
    if (g_usbSimulatedDisconnect || !IS_HANDLE_VALID(this->m_devHandle)) {
      Util::DriverLog("No dev for descriptor");
      return 1;
    }

    memset(pDest, 0, sizeof(libusb_device_descriptor));

    libusb_device_descriptor desc;
    if (libusb_get_device_descriptor(libusb_get_device(this->m_devHandle), &desc) == 0) {
      memcpy(pDest, &desc, sizeof(libusb_device_descriptor));

      return 0;
    }

    Util::DriverLog("Failed to get descriptor");
    return 1;
  }

  void CaesarUsbThread::ActivateInterfaceHelper(CaesarUsbThread* thisptr) {
    thisptr->m_usbCtx = g_usbCtx;

    uint8_t interfaceNum = thisptr->GetInterface();

    {
      std::lock_guard<std::mutex> lock(g_devHandleMutex);
      if (g_usbSimulatedDisconnect) {
        // Do not attempt to use the device if disconnect is simulated
        thisptr->m_devHandle = INVALID_DEVICE_HANDLE;
        return;
      }

      if (IS_HANDLE_VALID(g_devHandle)) {
        // Ping the device to see if it's still physically responding.
        uint16_t device_status = 0;
        int result = 0;
        {
          std::shared_lock<std::shared_mutex> pingLock(g_pendingOperationsMutex);
          result = libusb_control_transfer(
              g_devHandle,
              LIBUSB_REQUEST_TYPE_STANDARD,
              LIBUSB_REQUEST_GET_STATUS,
              0,
              0,
              reinterpret_cast<unsigned char*>(&device_status),
              sizeof(device_status),
              100
          );
        }

        bool device_present = (result >= 0);

        if (!device_present) {
          Util::DriverLog("Device disconnected. Closing existing handle.");

          // Wait for all pending operations to finish with an exclusive lock
          std::unique_lock<std::shared_mutex> opsLock(g_pendingOperationsMutex);
          libusb_close(g_devHandle);
          g_devHandle = INVALID_DEVICE_HANDLE;
        }
      }

      if (!IS_HANDLE_VALID(g_devHandle) && IS_HANDLE_VALID(g_usbCtx)) {
        g_devHandle = libusb_open_device_with_vid_pid(g_usbCtx, 0x054C, 0x0cde);
        if (g_devHandle == nullptr) {
          g_devHandle = INVALID_DEVICE_HANDLE;
        }
      }
    }

    thisptr->m_devHandle = g_devHandle;

    Util::DriverLog("Opened {} with handle {}", interfaceNum, (uint64_t)thisptr->m_devHandle);

    if (IS_HANDLE_VALID(thisptr->m_devHandle)) {
      // Doesn't matter on Windows, but we'll do it anyway since Ignition will need this.
      if (libusb_kernel_driver_active(thisptr->m_devHandle, interfaceNum) == 1) {
        libusb_detach_kernel_driver(thisptr->m_devHandle, interfaceNum);
      }

      if (libusb_claim_interface(thisptr->m_devHandle, interfaceNum) == 0) {
        if (interfaceNum == 7) {
          libusb_set_interface_alt_setting(thisptr->m_devHandle, interfaceNum, 1);
        }

        Util::DriverLog("Claimed interface {} successfully", interfaceNum);
        thisptr->OnConnected();
        Util::DriverLog("Configured interface {} successfully", interfaceNum);
        thisptr->m_state = 2; // Connected
        thisptr->m_winUsbActive = 1;
      } else {
        Util::DriverLog("Failed to claim interface {}. Closing handle.", interfaceNum);
        thisptr->m_devHandle = INVALID_DEVICE_HANDLE;
      }
    }
  }

  void CaesarUsbThread::DestructorHook(CaesarUsbThread* thisptr, bool shouldFree) {
    if (thisptr->m_winUsbActive != 0) {
      if (IS_HANDLE_VALID(thisptr->m_devHandle)) {
        std::lock_guard<std::mutex> lock(g_devHandleMutex);
        if (thisptr->m_devHandle == g_devHandle && IS_HANDLE_VALID(g_devHandle)) {
          libusb_release_interface(thisptr->m_devHandle, thisptr->GetInterface());
        }
        thisptr->m_devHandle = INVALID_DEVICE_HANDLE;
      }
      if (IS_HANDLE_VALID(thisptr->m_usbCtx)) {
        thisptr->m_usbCtx = INVALID_CONTEXT_HANDLE;
      }
      thisptr->m_winUsbActive = 0;
    }

    if (orig_destructor) orig_destructor(thisptr, shouldFree);
  }

  void CaesarUsbThread::ThreadLoopHook(CaesarUsbThread* thisptr) {
    while (thisptr->m_threadRunning == 0 && thisptr->m_stopRequested == 0) {
      if (thisptr->m_state == 1) { // Disconnected State
        ActivateInterfaceHelper(thisptr);

        if (thisptr->m_state != 2) {
          // Wait a bit before we try again.
          Sleep(500);
        }
      }
      else if (thisptr->m_state == 2) { // Connected State
        int result = thisptr->PollAndProcess();

        if (result != 0) {
          Util::DriverLog("PollAndProcess failed for interface {}. Disconnecting. Result: {}", thisptr->GetInterface(), result);

          thisptr->OnDisconnect();
          thisptr->m_state = 1;

          if (IS_HANDLE_VALID(thisptr->m_devHandle)) {
            std::lock_guard<std::mutex> lock(g_devHandleMutex);
            if (thisptr->m_devHandle == g_devHandle && IS_HANDLE_VALID(g_devHandle)) {
              libusb_release_interface(thisptr->m_devHandle, thisptr->GetInterface());
            }
            thisptr->m_devHandle = INVALID_DEVICE_HANDLE;
          }
          thisptr->m_winUsbActive = 0;

          // Wait a bit before we try again.
          Sleep(500);
        }
      }
    }

    if (IS_HANDLE_VALID(thisptr->m_devHandle)) {
      std::lock_guard<std::mutex> lock(g_devHandleMutex);
      if (thisptr->m_devHandle == g_devHandle && IS_HANDLE_VALID(g_devHandle)) {
        libusb_release_interface(thisptr->m_devHandle, thisptr->GetInterface());
      }
      thisptr->m_devHandle = INVALID_DEVICE_HANDLE;
    }
    
    thisptr->m_usbCtx = INVALID_CONTEXT_HANDLE;
    thisptr->m_winUsbActive = 0;
    thisptr->m_state = 0;
  }

  void CaesarUsbThread::JoinThreadHook(CaesarUsbThread* thisptr) {
    OutputDebugStringA("Joining interface thread");
    if (orig_joinThread) {
        orig_joinThread(thisptr);
    }
  }

  void CaesarUsbThread::StartHook(CaesarUsbThread* thisptr, bool connectNow) {
    thisptr->m_state = 1;
    thisptr->m_stopRequested = 0;
    thisptr->m_usbCtx = INVALID_CONTEXT_HANDLE;
    thisptr->m_devHandle = INVALID_DEVICE_HANDLE;

    if (connectNow) {
      Util::DriverLog("Connecting NOW for {}", thisptr->GetInterface());
      ActivateInterfaceHelper(thisptr);
    }
  }

  uint8_t CaesarUsbThread::GetInterfaceHook(CaesarUsbThread* thisptr) {
    if (orig_getInterface) {
      return orig_getInterface(thisptr);
    }
    return 0;
  }

  uint8_t CaesarUsbThread::GetEndpointHook(CaesarUsbThread* thisptr) {
    if (orig_getEndpoint) {
      return orig_getEndpoint(thisptr);
    }
    return 0;
  }

  void CaesarUsbThread::OnConnectedHook(CaesarUsbThread* thisptr) {
    if (orig_onConnected) {
      orig_onConnected(thisptr);
    }
  }

  void CaesarUsbThread::OnDisconnectHook(CaesarUsbThread* thisptr) {
    if (orig_onDisconnect) {
      orig_onDisconnect(thisptr);
    }
  }

  int CaesarUsbThread::PollAndProcessHook(CaesarUsbThread* thisptr) {
    if (orig_pollAndProcess) {
      return orig_pollAndProcess(thisptr);
    }
    return 0;
  }

  int CaesarUsbThread::ReadPipeHook(CaesarUsbThread* thisptr, uint8_t pipeId, char* buffer, size_t length) {
    return thisptr->TransferPipe(pipeId, buffer, length);
  }

  int CaesarUsbThread::ControlCommandHook(CaesarUsbThread* thisptr, uint8_t bIsSet, uint16_t reportId, void* buffer, uint16_t length, uint16_t value, uint16_t index, uint16_t subcmd) {
    return thisptr->ControlCommand(bIsSet, reportId, buffer, length, value, index, subcmd);
  }

  int CaesarUsbThread::GetDescriptorHook(CaesarUsbThread* thisptr, libusb_device_descriptor* pDest) {
    return thisptr->GetDescriptor(pDest);
  }
  
  void CaesarUsbThread::InstallHooks() {
    static HmdDriverLoader* pHmdDriverLoader = HmdDriverLoader::Instance();
    uintptr_t baseAddr = pHmdDriverLoader->GetBaseAddress();

    libusb_init(&g_usbCtx);

    HookLib::InstallHook(reinterpret_cast<void*>(baseAddr + 0x1225a0),
                         reinterpret_cast<void*>(DestructorHook),
                         reinterpret_cast<void**>(&orig_destructor));

    HookLib::InstallHook(reinterpret_cast<void*>(baseAddr + 0x127b70),
                         reinterpret_cast<void*>(ThreadLoopHook),
                         reinterpret_cast<void**>(&orig_threadLoop));

    HookLib::InstallHook(reinterpret_cast<void*>(baseAddr + 0x16b540),
                         reinterpret_cast<void*>(JoinThreadHook),
                         reinterpret_cast<void**>(&orig_joinThread));

    HookLib::InstallHook(reinterpret_cast<void*>(baseAddr + 0x125bb0),
                         reinterpret_cast<void*>(StartHook),
                         reinterpret_cast<void**>(&orig_start));

    HookLib::InstallHook(reinterpret_cast<void*>(baseAddr + 0x124cd0),
                         reinterpret_cast<void*>(GetInterfaceHook),
                         reinterpret_cast<void**>(&orig_getInterface));

    HookLib::InstallHook(reinterpret_cast<void*>(baseAddr + 0x124f60),
                         reinterpret_cast<void*>(GetEndpointHook),
                         reinterpret_cast<void**>(&orig_getEndpoint));

    HookLib::InstallHook(reinterpret_cast<void*>(baseAddr + 0x125660),
                         reinterpret_cast<void*>(OnConnectedHook),
                         reinterpret_cast<void**>(&orig_onConnected));

    HookLib::InstallHook(reinterpret_cast<void*>(baseAddr + 0x125b40),
                         reinterpret_cast<void*>(OnDisconnectHook),
                         reinterpret_cast<void**>(&orig_onDisconnect));

    HookLib::InstallHook(reinterpret_cast<void*>(baseAddr + 0x125cb0),
                         reinterpret_cast<void*>(PollAndProcessHook),
                         reinterpret_cast<void**>(&orig_pollAndProcess));

    HookLib::InstallHook(reinterpret_cast<void*>(baseAddr + 0x127d60),
                         reinterpret_cast<void*>(ReadPipeHook),
                         reinterpret_cast<void**>(&orig_read));

    HookLib::InstallHook(reinterpret_cast<void*>(baseAddr + 0x1283f0),
                         reinterpret_cast<void*>(ControlCommandHook),
                         reinterpret_cast<void**>(&orig_report));

    HookLib::InstallHook(reinterpret_cast<void*>(baseAddr + 0x124510),
                         reinterpret_cast<void*>(GetDescriptorHook),
                         reinterpret_cast<void**>(&orig_getDescriptor));

    // NOP some WinUSB calls for changing alternate settings for interface 7
    Util::SetInstructionNOPAtAddress(reinterpret_cast<void*>(baseAddr + 0x125926), 5);
    Util::SetInstructionNOPAtAddress(reinterpret_cast<void*>(baseAddr + 0x125952), 5);

    // NOP AbortPipe calls
    Util::SetInstructionNOPAtAddress(reinterpret_cast<void*>(baseAddr + 0x122158), 5);
    Util::SetInstructionNOPAtAddress(reinterpret_cast<void*>(baseAddr + 0x1221d8), 5);
    Util::SetInstructionNOPAtAddress(reinterpret_cast<void*>(baseAddr + 0x122258), 5);
    Util::SetInstructionNOPAtAddress(reinterpret_cast<void*>(baseAddr + 0x1222d8), 5);
    Util::SetInstructionNOPAtAddress(reinterpret_cast<void*>(baseAddr + 0x122368), 5);
    Util::SetInstructionNOPAtAddress(reinterpret_cast<void*>(baseAddr + 0x1223e8), 5);
    Util::SetInstructionNOPAtAddress(reinterpret_cast<void*>(baseAddr + 0x122478), 5);
    Util::SetInstructionNOPAtAddress(reinterpret_cast<void*>(baseAddr + 0x12262f), 5);
    Util::SetInstructionNOPAtAddress(reinterpret_cast<void*>(baseAddr + 0x1226cf), 5);
    Util::SetInstructionNOPAtAddress(reinterpret_cast<void*>(baseAddr + 0x12276f), 5);
    Util::SetInstructionNOPAtAddress(reinterpret_cast<void*>(baseAddr + 0x12280f), 5);
    Util::SetInstructionNOPAtAddress(reinterpret_cast<void*>(baseAddr + 0x1228bf), 5);
    Util::SetInstructionNOPAtAddress(reinterpret_cast<void*>(baseAddr + 0x12295f), 5);
    Util::SetInstructionNOPAtAddress(reinterpret_cast<void*>(baseAddr + 0x122a0f), 5);
    Util::SetInstructionNOPAtAddress(reinterpret_cast<void*>(baseAddr + 0x12395e), 5);
    Util::SetInstructionNOPAtAddress(reinterpret_cast<void*>(baseAddr + 0x1239ae), 5);
    Util::SetInstructionNOPAtAddress(reinterpret_cast<void*>(baseAddr + 0x1239fe), 5);
    Util::SetInstructionNOPAtAddress(reinterpret_cast<void*>(baseAddr + 0x123a4e), 5);
    Util::SetInstructionNOPAtAddress(reinterpret_cast<void*>(baseAddr + 0x123a9e), 5);
    Util::SetInstructionNOPAtAddress(reinterpret_cast<void*>(baseAddr + 0x123aee), 5);
    Util::SetInstructionNOPAtAddress(reinterpret_cast<void*>(baseAddr + 0x123b3e), 5);
    
  }

} // namespace psvr2_toolkit