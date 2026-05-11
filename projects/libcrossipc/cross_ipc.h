#pragma once
#include <cstddef>
#include <cstdint>

#if (defined(_WIN32) && !defined(__WINE__))
  #define WINDOWS_IPC
#endif

#ifdef WINDOWS_IPC
  #ifdef libcrossipc_EXPORTS
    #define CROSS_IPC_API __declspec(dllexport)
  #else
    #define CROSS_IPC_API __declspec(dllimport)
  #endif
#else
  #define CROSS_IPC_API __attribute__((visibility("default"))) __cdecl
#endif

class IIpcMutex {
public:
  virtual ~IIpcMutex() = default;
  virtual void lock() = 0;
  virtual bool try_lock() = 0;
  virtual void unlock() = 0;
};

class IIpcEvent {
public:
  virtual ~IIpcEvent() = default;
  virtual void set() = 0;
  virtual bool wait(uint32_t timeoutMs = 0xFFFFFFFF) = 0;
};

class IIpcSharedMemory {
public:
  virtual ~IIpcSharedMemory() = default;
  virtual void *map() = 0;
  virtual void unmap() = 0;
};

class IIpcBroadcast {
public:
  virtual ~IIpcBroadcast() = default;
  virtual bool wait(uint32_t timeoutMs = 0xFFFFFFFF) = 0;
  virtual void notify_all() = 0;
};

extern "C" {
CROSS_IPC_API IIpcMutex *CreateIpcMutex(const char *name);
CROSS_IPC_API IIpcEvent *CreateIpcEvent(const char *name);
CROSS_IPC_API IIpcSharedMemory *CreateIpcSharedMemory(const char *name,
                                                      size_t size);
CROSS_IPC_API IIpcBroadcast *CreateIpcBroadcast(const char *name);

CROSS_IPC_API void DestroyIpcMutex(IIpcMutex *mutex);
CROSS_IPC_API void IpcMutex_Lock(IIpcMutex *mutex);
CROSS_IPC_API bool IpcMutex_TryLock(IIpcMutex *mutex);
CROSS_IPC_API void IpcMutex_Unlock(IIpcMutex *mutex);

CROSS_IPC_API void DestroyIpcEvent(IIpcEvent *event);
CROSS_IPC_API void IpcEvent_Set(IIpcEvent *event);
CROSS_IPC_API bool IpcEvent_Wait(IIpcEvent *event, uint32_t timeoutMs);

CROSS_IPC_API void DestroyIpcSharedMemory(IIpcSharedMemory *shm);
CROSS_IPC_API void *IpcSharedMemory_Map(IIpcSharedMemory *shm);
CROSS_IPC_API void IpcSharedMemory_Unmap(IIpcSharedMemory *shm);

CROSS_IPC_API void DestroyIpcBroadcast(IIpcBroadcast *broadcast);
CROSS_IPC_API bool IpcBroadcast_Wait(IIpcBroadcast *broadcast, uint32_t timeoutMs);
CROSS_IPC_API void IpcBroadcast_NotifyAll(IIpcBroadcast *broadcast);
}
