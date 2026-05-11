#pragma once

#if defined(_WIN32)

#include <string>
#include <windows.h>

#include "cross_ipc.h"

#define MAX_CLIENTS 12


class WindowsIpcMutex : public IIpcMutex {
public:
  explicit WindowsIpcMutex(const char *name);
  virtual ~WindowsIpcMutex();

  void lock() override;
  bool try_lock() override;
  void unlock() override;

private:
  HANDLE m_hMutex;
};

class WindowsIpcEvent : public IIpcEvent {
public:
  explicit WindowsIpcEvent(const char *name);
  virtual ~WindowsIpcEvent();

  void set() override;
  bool wait(uint32_t timeoutMs = 0xFFFFFFFF) override;

private:
  HANDLE m_hEvent;
};

class WindowsIpcSharedMemory : public IIpcSharedMemory {
public:
  WindowsIpcSharedMemory(const char *name, size_t size);
  virtual ~WindowsIpcSharedMemory();

  void *map() override;
  void unmap() override;

private:
  HANDLE m_hFileMapping;
  void *m_pBuffer;
};

struct BroadcastWaiter {
  char eventName[128];
  DWORD clientPid;
  bool active;
};

struct BroadcastSharedData {
  BroadcastWaiter waiters[MAX_CLIENTS];
};

struct CachedClient {
  HANDLE hEvent;
  HANDLE hProcess;
};

class WindowsIpcBroadcast : public IIpcBroadcast {
public:
  explicit WindowsIpcBroadcast(const char *name);
  virtual ~WindowsIpcBroadcast();

  bool wait(uint32_t timeoutMs = 0xFFFFFFFF) override;
  void notify_all() override;

private:
  std::string m_baseName;
  HANDLE m_hMutex;
  HANDLE m_hFileMapping;
  BroadcastSharedData *m_pData;

  HANDLE m_hLocalEvent;
  int m_slot;

  CachedClient m_cachedClients[MAX_CLIENTS];
};

#endif // _WIN32