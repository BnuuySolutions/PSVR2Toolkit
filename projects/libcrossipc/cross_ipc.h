#pragma once
#include <cstddef>
#include <cstdint>

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
}
