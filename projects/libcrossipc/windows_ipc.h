#pragma once

#ifdef _WIN32

#include <string>
#include <windows.h>

#include "cross_ipc.h"


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

struct BroadcastSharedData {
  uint32_t counter;
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

  static constexpr uint32_t NUM_EVENTS = 4;
  HANDLE m_hEvents[NUM_EVENTS];
};

#endif // _WIN32