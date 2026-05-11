#pragma once

#if defined(__linux__)

#include <string>
#include <pthread.h>
#include <atomic>

#include "cross_ipc.h"

class LinuxIpcMutex : public IIpcMutex {
private:
    std::string m_name;
    int m_fd;
    pthread_mutex_t* m_mutex;

public:
    LinuxIpcMutex(const char* name);
    ~LinuxIpcMutex() override;
    void lock() override;
    bool try_lock() override;
    void unlock() override;
};

class LinuxIpcEvent : public IIpcEvent {
private:
    std::string m_name;
    int m_fd;
    struct EventData;
    EventData *m_data;

public:
    LinuxIpcEvent(const char* name);
    ~LinuxIpcEvent() override;
    void set() override;
    bool wait(uint32_t timeoutMs = 0xFFFFFFFF) override;
};

class LinuxIpcSharedMemory : public IIpcSharedMemory {
private:
    std::string m_name;
    int m_fd;
    size_t m_size;
    void *m_ptr;

public:
    LinuxIpcSharedMemory(const char *name, size_t size);
    ~LinuxIpcSharedMemory() override;
    void *map() override;
    void unmap() override;
};

class LinuxIpcBroadcast : public IIpcBroadcast {
private:
    std::string m_name;
    int m_fd;
    struct BroadcastData;
    BroadcastData *m_data;
    uint32_t m_last_val;

public:
    LinuxIpcBroadcast(const char *name);
    ~LinuxIpcBroadcast() override;
    bool wait(uint32_t timeoutMs = 0xFFFFFFFF) override;
    void notify_all() override;
};

#endif