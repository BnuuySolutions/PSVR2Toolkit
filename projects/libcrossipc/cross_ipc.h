#pragma once
#include <cstddef>

#ifdef _WIN32
  #ifdef libcrossipc_EXPORTS
    #define CROSS_IPC_API __declspec(dllexport)
  #else
    #define CROSS_IPC_API __declspec(dllimport)
  #endif
#else
  #define CROSS_IPC_API __attribute__((visibility("default")))
#endif

class IIpcMutex {
public:
    virtual ~IIpcMutex() = default;
    virtual void lock() = 0;
    virtual void unlock() = 0;
};

class IIpcEvent {
public:
    virtual ~IIpcEvent() = default;
    virtual void set() = 0;
    virtual void wait() = 0;
};

class IIpcSharedMemory {
public:
    virtual ~IIpcSharedMemory() = default;
    virtual void* map() = 0;
    virtual void unmap() = 0;
};

// Factory functions to be exported by your OS abstraction DLL
extern "C" {
    CROSS_IPC_API IIpcMutex* CreateIpcMutex(const char* name);
    CROSS_IPC_API IIpcEvent* CreateIpcEvent(const char* name);
    CROSS_IPC_API IIpcSharedMemory* CreateIpcSharedMemory(const char* name, size_t size);
}
