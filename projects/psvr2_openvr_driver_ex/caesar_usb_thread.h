#pragma once

#include <libusb-1.0/libusb.h>
#include <windows.h>
#include <winusb.h>
#include <cstdint>

namespace psvr2_toolkit {

#pragma pack(push, 1)
  class CaesarUsbThread {
  public:
    virtual ~CaesarUsbThread();          
    virtual void ThreadLoop();           
    virtual void JoinThread();           
    virtual void Start(char connectNow);                
    virtual uint8_t GetInterface();      
    virtual uint8_t GetEndpoint();       
    virtual void ConfigEndpoint();       
    virtual void OnDisconnect();         
    virtual int PollAndProcess();        

    void* m_pThreadData;                 // 0x08
    uint8_t m_threadRunning;             // 0x10
    uint8_t _pad1[0x17];                 // 0x11
    
    int32_t m_state;                     // 0x28 (1 = Disconnected, 2 = Connected)
    uint32_t _pad2;                      // 0x2C
    
    void* m_mutex;                       // 0x30
    uint8_t _pad3[0x08];                 // 0x38
    
    int32_t m_winUsbActive;              // 0x40
    uint32_t _pad4;                      // 0x44

    // These were WinUSB fields
    libusb_context* m_usbCtx;            // 0x48 (param_1[9]) 
    libusb_device_handle* m_devHandle;   // 0x50 (param_1[10])
    
    uint8_t _pad5[0x188];                // 0x58
    
    uint8_t m_stopRequested;             // 0x1E0
    uint8_t _pad6[0x37];                 // 0x1E1

    uint32_t m_lastError;                // 0x218

    static void ActivateInterfaceHelper(CaesarUsbThread* thisptr);
    
    // Hooks have to go here. We also put implementations here, or else we go in a loop with vtables.
    // And we have to basically not touch vtables to pull this off.
    static void DestructorHook(CaesarUsbThread* thisptr, char a2);
    static void ThreadLoopHook(CaesarUsbThread* thisptr);
    static void JoinThreadHook(CaesarUsbThread* thisptr);
    static void StartHook(CaesarUsbThread* thisptr, char connectNow);
    static uint8_t GetInterfaceHook(CaesarUsbThread* thisptr);
    static uint8_t GetEndpointHook(CaesarUsbThread* thisptr);
    static void ConfigEndpointHook(CaesarUsbThread* thisptr);
    static void OnDisconnectHook(CaesarUsbThread* thisptr);
    static int PollAndProcessHook(CaesarUsbThread* thisptr);

    static int ReadPipeHook(CaesarUsbThread* thisptr, uint8_t pipeId, char* buffer, size_t length);
    static int ControlTransferHook(CaesarUsbThread* thisptr, uint8_t bIsSet, uint16_t reportId, void* buffer, uint16_t length, uint16_t value, uint16_t index, uint16_t subcmd);
    static int GetDescriptorHook(CaesarUsbThread* thisptr, void* pDest);

    static void InstallHooks();

  private:
    static void (*orig_destructor)(CaesarUsbThread* thisptr, char a2);
    static void (*orig_threadLoop)(CaesarUsbThread* thisptr);
    static void (*orig_joinThread)(CaesarUsbThread* thisptr);
    static void (*orig_start)(CaesarUsbThread* thisptr);
    static uint8_t (*orig_getInterface)(CaesarUsbThread* thisptr);
    static uint8_t (*orig_getEndpoint)(CaesarUsbThread* thisptr);
    static void (*orig_configEndpoint)(CaesarUsbThread* thisptr);
    static void (*orig_onDisconnect)(CaesarUsbThread* thisptr);
    static int (*orig_pollAndProcess)(CaesarUsbThread* thisptr);
    
    static int (*orig_read)(CaesarUsbThread* thisptr, uint8_t pipeId, char* buffer, size_t length);
    static int (*orig_report)(CaesarUsbThread* thisptr, uint8_t bIsSet, uint16_t reportId, void* buffer, uint16_t length, uint16_t value, uint16_t index, uint16_t subcmd);
    static bool (*orig_getDescriptor)(CaesarUsbThread* thisptr, void* pDest);
  };
#pragma pack(pop)

} // namespace psvr2_toolkit