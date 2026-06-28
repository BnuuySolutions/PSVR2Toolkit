#ifdef _WIN32

#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>

#include "windows_ipc.h"


WindowsIpcMutex::WindowsIpcMutex(const char *name) {
  m_hMutex = CreateMutexA(NULL, FALSE, name);
  if (!m_hMutex) {
    throw std::runtime_error("CreateMutexA failed. Error: " +
                             std::to_string(GetLastError()));
  }
}

WindowsIpcMutex::~WindowsIpcMutex() {
  if (m_hMutex) {
    CloseHandle(m_hMutex);
  }
}

void WindowsIpcMutex::lock() {
  if (WaitForSingleObject(m_hMutex, INFINITE) == WAIT_FAILED) {
    throw std::runtime_error("WaitForSingleObject failed on mutex. Error: " +
                             std::to_string(GetLastError()));
  }
}

bool WindowsIpcMutex::try_lock() {
  DWORD result = WaitForSingleObject(m_hMutex, 0);
  // WAIT_ABANDONED means the previous owning process crashed, and we
  // successfully claimed this mutex.
  return (result == WAIT_OBJECT_0 || result == WAIT_ABANDONED);
}

void WindowsIpcMutex::unlock() {
  if (!ReleaseMutex(m_hMutex)) {
    throw std::runtime_error("ReleaseMutex failed. Error: " +
                             std::to_string(GetLastError()));
  }
}

WindowsIpcEvent::WindowsIpcEvent(const char *name) {
  m_hEvent = CreateEventA(NULL, FALSE, FALSE, name);
  if (!m_hEvent) {
    throw std::runtime_error("CreateEventA failed. Error: " +
                             std::to_string(GetLastError()));
  }
}

WindowsIpcEvent::~WindowsIpcEvent() {
  if (m_hEvent) {
    CloseHandle(m_hEvent);
  }
}

void WindowsIpcEvent::set() {
  if (!SetEvent(m_hEvent)) {
    throw std::runtime_error("SetEvent failed. Error: " +
                             std::to_string(GetLastError()));
  }
}

bool WindowsIpcEvent::wait(uint32_t timeoutMs) {
  DWORD result = WaitForSingleObject(m_hEvent, timeoutMs);
  if (result == WAIT_FAILED) {
    throw std::runtime_error("WaitForSingleObject failed on event. Error: " +
                             std::to_string(GetLastError()));
  }
  return result == WAIT_OBJECT_0;
}

WindowsIpcSharedMemory::WindowsIpcSharedMemory(const char *name, size_t size)
    : m_pBuffer(nullptr) {
  m_hFileMapping =
      CreateFileMappingA(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0,
                         static_cast<DWORD>(size), name);
  if (!m_hFileMapping) {
    throw std::runtime_error("CreateFileMappingA failed. Error: " +
                             std::to_string(GetLastError()));
  }
}

WindowsIpcSharedMemory::~WindowsIpcSharedMemory() {
  unmap();
  if (m_hFileMapping) {
    CloseHandle(m_hFileMapping);
  }
}

void *WindowsIpcSharedMemory::map() {
  if (!m_pBuffer && m_hFileMapping) {
    m_pBuffer = MapViewOfFile(m_hFileMapping, FILE_MAP_ALL_ACCESS, 0, 0, 0);
    if (!m_pBuffer) {
      throw std::runtime_error("MapViewOfFile failed. Error: " +
                               std::to_string(GetLastError()));
    }
  }
  return m_pBuffer;
}

void WindowsIpcSharedMemory::unmap() {
  if (m_pBuffer) {
    UnmapViewOfFile(m_pBuffer);
    m_pBuffer = nullptr;
  }
}

WindowsIpcBroadcast::WindowsIpcBroadcast(const char *name)
    : m_baseName(name) {
  std::string mutexName = m_baseName + "_BCAST_MTX";
  std::string shmName = m_baseName + "_BCAST_SHM";

  m_hMutex = CreateMutexA(NULL, FALSE, mutexName.c_str());
  if (!m_hMutex) {
    throw std::runtime_error("CreateMutexA failed on broadcast. Error: " +
                             std::to_string(GetLastError()));
  }

  m_hFileMapping =
      CreateFileMappingA(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0,
                         sizeof(BroadcastSharedData), shmName.c_str());
  if (!m_hFileMapping) {
    throw std::runtime_error("CreateFileMappingA failed on broadcast. Error: " +
                             std::to_string(GetLastError()));
  }

  m_pData = static_cast<BroadcastSharedData *>(MapViewOfFile(
      m_hFileMapping, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(BroadcastSharedData)));
  if (!m_pData) {
    throw std::runtime_error("MapViewOfFile failed on broadcast. Error: " +
                             std::to_string(GetLastError()));
  }

  for (uint32_t i = 0; i < NUM_EVENTS; i++) {
    std::string eventName = m_baseName + "_BCAST_EVT_" + std::to_string(i);
    m_hEvents[i] = CreateEventA(NULL, TRUE, i == 0 ? FALSE : TRUE, eventName.c_str());
    if (!m_hEvents[i]) {
      throw std::runtime_error("CreateEventA failed on broadcast. Error: " +
                               std::to_string(GetLastError()));
    }
  }
}

WindowsIpcBroadcast::~WindowsIpcBroadcast() {
  for (uint32_t i = 0; i < NUM_EVENTS; i++) {
    if (m_hEvents[i])
      CloseHandle(m_hEvents[i]);
  }
  if (m_pData)
    UnmapViewOfFile(m_pData);
  if (m_hFileMapping)
    CloseHandle(m_hFileMapping);
  if (m_hMutex)
    CloseHandle(m_hMutex);
}

bool WindowsIpcBroadcast::wait(uint32_t timeoutMs) {
  if (timeoutMs == 0) {
    return false;
  }

  DWORD waitRes = WaitForSingleObject(m_hMutex, INFINITE);
  if (waitRes != WAIT_OBJECT_0) {
    throw std::runtime_error(
        "WaitForSingleObject failed on mutex in broadcast wait start. Error: " +
        std::to_string(GetLastError()));
  }

  uint32_t startCount = m_pData->counter;

  DWORD result = SignalObjectAndWait(m_hMutex, m_hEvents[startCount % NUM_EVENTS], timeoutMs, FALSE);

  if (waitRes != WAIT_OBJECT_0 && waitRes != WAIT_TIMEOUT) {
    ReleaseMutex(m_hMutex);
    throw std::runtime_error(
        "SignalObjectAndWait failed on broadcast wait. Error: " +
        std::to_string(GetLastError()));
  }

  waitRes = WaitForSingleObject(m_hMutex, INFINITE);
  if (waitRes != WAIT_OBJECT_0) {
    throw std::runtime_error(
        "WaitForSingleObject failed on mutex in broadcast wait finish. Error: " +
        std::to_string(GetLastError()));
  }

  bool signaled = m_pData->counter != startCount;

  ReleaseMutex(m_hMutex);
  return signaled;
}

void WindowsIpcBroadcast::notify_all() {
  DWORD waitRes = WaitForSingleObject(m_hMutex, INFINITE);
  if (waitRes != WAIT_OBJECT_0) {
    throw std::runtime_error(
        "WaitForSingleObject failed on mutex in broadcast notify_all. Error: " +
        std::to_string(GetLastError()));
  }

  uint32_t prevCount = m_pData->counter;
  m_pData->counter++;

  // Reset the next one, and then signal the current event.
  ResetEvent(m_hEvents[m_pData->counter % NUM_EVENTS]);
  SetEvent(m_hEvents[prevCount % NUM_EVENTS]); 

  ReleaseMutex(m_hMutex);
}

#endif // _WIN32