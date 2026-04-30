#include "cross_ipc.h"
#include "windows_ipc.h"

extern "C" {
    IIpcMutex* CreateIpcMutex(const char* name) {
        return new WindowsIpcMutex(name);
    }
    IIpcEvent* CreateIpcEvent(const char* name) {
        return new WindowsIpcEvent(name);
    }
    IIpcSharedMemory* CreateIpcSharedMemory(const char* name, size_t size) {
        return new WindowsIpcSharedMemory(name, size);
    }
}