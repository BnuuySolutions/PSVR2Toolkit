#include <windows.h>

#include "cross_ipc.h"

class WindowsIpcMutex : public IIpcMutex {
public:
    explicit WindowsIpcMutex(const char* name);
    virtual ~WindowsIpcMutex();

    void lock() override;
    void unlock() override;

private:
    HANDLE m_hMutex;
};

class WindowsIpcEvent : public IIpcEvent {
public:
    explicit WindowsIpcEvent(const char* name);
    virtual ~WindowsIpcEvent();

    void set() override;
    void wait() override;

private:
    HANDLE m_hEvent;
};

class WindowsIpcSharedMemory : public IIpcSharedMemory {
public:
    WindowsIpcSharedMemory(const char* name, size_t size);
    virtual ~WindowsIpcSharedMemory();

    void* map() override;
    void unmap() override;

private:
    HANDLE m_hFileMapping;
    void* m_pBuffer;
};