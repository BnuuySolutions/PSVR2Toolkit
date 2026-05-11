#if defined(_WIN32)

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
    : m_baseName(name), m_slot(-1) {
  for (int i = 0; i < MAX_CLIENTS; i++) {
    m_cachedClients[i].hEvent = NULL;
    m_cachedClients[i].hProcess = NULL;
  }

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
  bool alreadyExists = (GetLastError() == ERROR_ALREADY_EXISTS);
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

  if (!alreadyExists) {
    memset(m_pData, 0, sizeof(BroadcastSharedData));
  }

  char evtName[128];
  snprintf(evtName, sizeof(evtName), "%s_EVT_%lu_%p", m_baseName.c_str(),
           GetCurrentProcessId(), this);

  m_hLocalEvent = CreateEventA(NULL, FALSE, FALSE, evtName);
  if (!m_hLocalEvent) {
    throw std::runtime_error("CreateEventA failed on broadcast. Error: " +
                             std::to_string(GetLastError()));
  }

  WaitForSingleObject(m_hMutex, INFINITE);
  for (int i = 0; i < MAX_CLIENTS; i++) {
    if (!m_pData->waiters[i].active) {
      m_slot = i;
      memcpy(m_pData->waiters[i].eventName, evtName,
             sizeof(m_pData->waiters[i].eventName));
      m_pData->waiters[i].clientPid = GetCurrentProcessId();
      m_pData->waiters[i].active = true;
      break;
    }
  }
  ReleaseMutex(m_hMutex);
}

WindowsIpcBroadcast::~WindowsIpcBroadcast() {
  if (m_slot != -1 && m_hMutex) {
    WaitForSingleObject(m_hMutex, INFINITE);
    m_pData->waiters[m_slot].active = false;
    ReleaseMutex(m_hMutex);
  }

  if (m_hLocalEvent)
    CloseHandle(m_hLocalEvent);

  for (int i = 0; i < MAX_CLIENTS; i++) {
    if (m_cachedClients[i].hEvent)
      CloseHandle(m_cachedClients[i].hEvent);
    if (m_cachedClients[i].hProcess)
      CloseHandle(m_cachedClients[i].hProcess);
  }

  if (m_pData)
    UnmapViewOfFile(m_pData);
  if (m_hFileMapping)
    CloseHandle(m_hFileMapping);
  if (m_hMutex)
    CloseHandle(m_hMutex);
}

bool WindowsIpcBroadcast::wait(uint32_t timeoutMs) {
  if (m_hLocalEvent) {
    DWORD result = WaitForSingleObject(m_hLocalEvent, timeoutMs);
    if (result == WAIT_FAILED) {
      throw std::runtime_error(
          "WaitForSingleObject failed on broadcast. Error: " +
          std::to_string(GetLastError()));
    }
    return result == WAIT_OBJECT_0;
  }
  return false;
}

void WindowsIpcBroadcast::notify_all() {
  WaitForSingleObject(m_hMutex, INFINITE);
  for (int i = 0; i < MAX_CLIENTS; i++) {
    if (m_pData->waiters[i].active) {
      if (!m_cachedClients[i].hEvent) {
        m_cachedClients[i].hEvent = OpenEventA(EVENT_MODIFY_STATE, FALSE,
                                               m_pData->waiters[i].eventName);
        if (m_cachedClients[i].hEvent) {
          m_cachedClients[i].hProcess =
              OpenProcess(SYNCHRONIZE, FALSE, m_pData->waiters[i].clientPid);
        } else if (GetLastError() == ERROR_FILE_NOT_FOUND) {
          m_pData->waiters[i].active = false;
        }
      }

      if (m_cachedClients[i].hEvent) {
        bool isAlive = true;
        if (m_cachedClients[i].hProcess &&
            WaitForSingleObject(m_cachedClients[i].hProcess, 0) ==
                WAIT_OBJECT_0) {
          isAlive = false;
        }

        if (isAlive) {
          SetEvent(m_cachedClients[i].hEvent);
        } else {
          m_pData->waiters[i].active =
              false; // Client crashed/exited, prune the slot
        }
      }
    }

    // Clean up our handles if the client is no longer active
    if (!m_pData->waiters[i].active && m_cachedClients[i].hEvent) {
      CloseHandle(m_cachedClients[i].hEvent);
      m_cachedClients[i].hEvent = NULL;
      if (m_cachedClients[i].hProcess) {
        CloseHandle(m_cachedClients[i].hProcess);
        m_cachedClients[i].hProcess = NULL;
      }
    }
  }
  ReleaseMutex(m_hMutex);
}

#endif // _WIN32