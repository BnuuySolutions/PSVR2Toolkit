#include "cross_ipc.h"

#ifdef WINDOWS_IPC
#include "windows_ipc.h"
#else
#include "linux_ipc.h"
#endif

extern "C" {
IIpcMutex *CreateIpcMutex(const char *name) {
#ifdef WINDOWS_IPC
  return new WindowsIpcMutex(name);
#else
  return new LinuxIpcMutex(name);
#endif
}
IIpcEvent *CreateIpcEvent(const char *name) {
#ifdef WINDOWS_IPC
  return new WindowsIpcEvent(name);
#else
  return new LinuxIpcEvent(name);
#endif
}
IIpcSharedMemory *CreateIpcSharedMemory(const char *name, size_t size) {
#ifdef WINDOWS_IPC
  return new WindowsIpcSharedMemory(name, size);
#else
  return new LinuxIpcSharedMemory(name, size);
#endif
}
IIpcBroadcast *CreateIpcBroadcast(const char *name) {
#ifdef WINDOWS_IPC
  return new WindowsIpcBroadcast(name);
#else
  return new LinuxIpcBroadcast(name);
#endif
}

void DestroyIpcMutex(IIpcMutex *mutex) {
  delete mutex;
}

void IpcMutex_Lock(IIpcMutex *mutex) {
  mutex->lock();
}

bool IpcMutex_TryLock(IIpcMutex *mutex) {
  return mutex->try_lock();
}

void IpcMutex_Unlock(IIpcMutex *mutex) {
  mutex->unlock();
}

void DestroyIpcEvent(IIpcEvent *event) {
  delete event;
}

void IpcEvent_Set(IIpcEvent *event) {
  event->set();
}

bool IpcEvent_Wait(IIpcEvent *event, uint32_t timeoutMs) {
  return event->wait(timeoutMs);
}

void DestroyIpcSharedMemory(IIpcSharedMemory *shm) {
  delete shm;
}

void *IpcSharedMemory_Map(IIpcSharedMemory *shm) {
  return shm->map();
}

void IpcSharedMemory_Unmap(IIpcSharedMemory *shm) {
  shm->unmap();
}

void DestroyIpcBroadcast(IIpcBroadcast *broadcast) {
  delete broadcast;
}

bool IpcBroadcast_Wait(IIpcBroadcast *broadcast, uint32_t timeoutMs) {
  return broadcast->wait(timeoutMs);
}

void IpcBroadcast_NotifyAll(IIpcBroadcast *broadcast) {
  broadcast->notify_all();
}
}