#include <stdexcept>
#include <string>

#include "windows_ipc.h"


WindowsIpcMutex::WindowsIpcMutex(const char* name) {
    m_hMutex = CreateMutexA(NULL, FALSE, name);
    if (!m_hMutex) {
        throw std::runtime_error("CreateMutexA failed. Error: " + std::to_string(GetLastError()));
    }
}

WindowsIpcMutex::~WindowsIpcMutex() {
    if (m_hMutex) {
        CloseHandle(m_hMutex);
    }
}

void WindowsIpcMutex::lock() {
    if (WaitForSingleObject(m_hMutex, INFINITE) == WAIT_FAILED) {
        throw std::runtime_error("WaitForSingleObject failed on mutex. Error: " + std::to_string(GetLastError()));
    }
}

bool WindowsIpcMutex::try_lock() {
    DWORD result = WaitForSingleObject(m_hMutex, 0);
    // WAIT_ABANDONED means the previous owning process crashed, and we successfully claimed this mutex.
    return (result == WAIT_OBJECT_0 || result == WAIT_ABANDONED);
}

void WindowsIpcMutex::unlock() {
    if (!ReleaseMutex(m_hMutex)) {
        throw std::runtime_error("ReleaseMutex failed. Error: " + std::to_string(GetLastError()));
    }
}


WindowsIpcEvent::WindowsIpcEvent(const char* name) {
    m_hEvent = CreateEventA(NULL, FALSE, FALSE, name);
    if (!m_hEvent) {
        throw std::runtime_error("CreateEventA failed. Error: " + std::to_string(GetLastError()));
    }
}

WindowsIpcEvent::~WindowsIpcEvent() {
    if (m_hEvent) {
        CloseHandle(m_hEvent);
    }
}

void WindowsIpcEvent::set() {
    if (!SetEvent(m_hEvent)) {
        throw std::runtime_error("SetEvent failed. Error: " + std::to_string(GetLastError()));
    }
}

void WindowsIpcEvent::wait() {
    if (WaitForSingleObject(m_hEvent, INFINITE) == WAIT_FAILED) {
        throw std::runtime_error("WaitForSingleObject failed on event. Error: " + std::to_string(GetLastError()));
    }
}


WindowsIpcSharedMemory::WindowsIpcSharedMemory(const char* name, size_t size) 
    : m_pBuffer(nullptr) {
    m_hFileMapping = CreateFileMappingA(
        INVALID_HANDLE_VALUE, 
        NULL, 
        PAGE_READWRITE, 
        0, 
        static_cast<DWORD>(size), 
        name
    );
    if (!m_hFileMapping) {
        throw std::runtime_error("CreateFileMappingA failed. Error: " + std::to_string(GetLastError()));
    }
}

WindowsIpcSharedMemory::~WindowsIpcSharedMemory() {
    unmap();
    if (m_hFileMapping) {
        CloseHandle(m_hFileMapping);
    }
}

void* WindowsIpcSharedMemory::map() {
    if (!m_pBuffer && m_hFileMapping) {
        m_pBuffer = MapViewOfFile(m_hFileMapping, FILE_MAP_ALL_ACCESS, 0, 0, 0);
        if (!m_pBuffer) {
            throw std::runtime_error("MapViewOfFile failed. Error: " + std::to_string(GetLastError()));
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