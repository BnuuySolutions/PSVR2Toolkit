#pragma once

#include <libusb-1.0/libusb.h>
#include <windows.h>
#include <winusb.h>
#include <cstdint>

#define INVALID_CONTEXT_HANDLE ((libusb_context*)-1)
#define INVALID_DEVICE_HANDLE ((libusb_device_handle*)-1)

namespace psvr2_toolkit {

#pragma pack(push, 1)
  class CaesarUsbThread {
  public:
    virtual ~CaesarUsbThread();
    virtual void ThreadLoop();
    virtual void JoinThread();
    virtual void Start(bool connectNow);
    virtual uint8_t GetInterface();
    virtual uint8_t GetEndpoint();
    virtual void OnConnected();
    virtual void OnDisconnect();
    virtual int PollAndProcess();

    int TransferPipe(uint8_t pipeId, char* buffer, size_t length);
    int ControlCommand(uint8_t bIsSet, uint16_t reportId, void* buffer, uint16_t length, uint16_t value, uint16_t index, uint16_t& subcmd);
    int ControlCommand(uint8_t bIsSet, uint16_t reportId, void* buffer, uint16_t length, uint16_t value, uint16_t index, uint16_t&& subcmd) {
      uint16_t temp = subcmd;
      return ControlCommand(bIsSet, reportId, buffer, length, value, index, temp);
    }
    int GetDescriptor(libusb_device_descriptor* pDest);

    void* m_pThreadData;
    uint8_t m_threadRunning = 0;
    uint8_t _pad1[0x17];

    int32_t m_state;
    uint32_t _pad2;

    void* m_mutex;
    uint8_t _pad3[0x08];

    int32_t m_winUsbActive = 0;
    uint32_t _pad4;

    // These were WinUSB fields. We have to use -1 to indicate an invalid handle.
    libusb_context* m_usbCtx = INVALID_CONTEXT_HANDLE;
    libusb_device_handle* m_devHandle = INVALID_DEVICE_HANDLE;

    uint8_t _pad5[0x188];

    uint8_t m_stopRequested = 0;
    uint8_t _pad6[0x37];

    uint32_t m_lastError = 0;

    static void ActivateInterfaceHelper(CaesarUsbThread* thisptr);
    static void SetUsbConnectionState(bool connected);

    // Hooks have to go here. We also put implementations here, or else we go in a loop with vtables.
    // And we have to basically not touch vtables to pull this off.
    static void DestructorHook(CaesarUsbThread* thisptr, bool shouldFree);
    static void ThreadLoopHook(CaesarUsbThread* thisptr);
    static void JoinThreadHook(CaesarUsbThread* thisptr);
    static void StartHook(CaesarUsbThread* thisptr, bool connectNow);
    static uint8_t GetInterfaceHook(CaesarUsbThread* thisptr);
    static uint8_t GetEndpointHook(CaesarUsbThread* thisptr);
    static void OnConnectedHook(CaesarUsbThread* thisptr);
    static void OnDisconnectHook(CaesarUsbThread* thisptr);
    static int PollAndProcessHook(CaesarUsbThread* thisptr);

    static int ReadPipeHook(CaesarUsbThread* thisptr, uint8_t pipeId, char* buffer, size_t length);
    static int ControlCommandHook(CaesarUsbThread* thisptr, uint8_t bIsSet, uint16_t reportId, void* buffer, uint16_t length, uint16_t value, uint16_t index, uint16_t subcmd);
    static int GetDescriptorHook(CaesarUsbThread* thisptr, libusb_device_descriptor* pDest);

    static void InstallHooks();
    static void Stop();

  private:
    static void (*orig_destructor)(CaesarUsbThread* thisptr, bool shouldFree);
    static void (*orig_threadLoop)(CaesarUsbThread* thisptr);
    static void (*orig_joinThread)(CaesarUsbThread* thisptr);
    static void (*orig_start)(CaesarUsbThread* thisptr, bool connectNow);
    static uint8_t (*orig_getInterface)(CaesarUsbThread* thisptr);
    static uint8_t (*orig_getEndpoint)(CaesarUsbThread* thisptr);
    static void (*orig_onConnected)(CaesarUsbThread* thisptr);
    static void (*orig_onDisconnect)(CaesarUsbThread* thisptr);
    static int (*orig_pollAndProcess)(CaesarUsbThread* thisptr);

    static int (*orig_read)(CaesarUsbThread* thisptr, uint8_t pipeId, char* buffer, size_t length);
    static int (*orig_report)(CaesarUsbThread* thisptr, uint8_t bIsSet, uint16_t reportId, void* buffer, uint16_t length, uint16_t value, uint16_t index, uint16_t subcmd);
    static bool (*orig_getDescriptor)(CaesarUsbThread* thisptr, libusb_device_descriptor* pDest);
  };
#pragma pack(pop)

} // namespace psvr2_toolkit