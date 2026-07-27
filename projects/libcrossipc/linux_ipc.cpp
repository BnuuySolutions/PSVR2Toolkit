#ifdef __linux__

#include <climits>
#include <fcntl.h>
#include <linux/futex.h>
#include <stdexcept>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <sys/time.h>
#include <unistd.h>

#include "linux_ipc.h"

LinuxIpcMutex::LinuxIpcMutex(const char *name) : m_name(name), m_fd(-1), m_mutex(nullptr) {
  std::string shmName = "/" + m_name + "_MTX_SHM";
  m_fd = shm_open(shmName.c_str(), O_CREAT | O_EXCL | O_RDWR, 0666);
  bool initialize = false;

  if (m_fd != -1) {
    initialize = true;
    if (ftruncate(m_fd, sizeof(pthread_mutex_t)) == -1)
      throw std::runtime_error("ftruncate failed");
  } else {
    m_fd = shm_open(shmName.c_str(), O_RDWR, 0666);
    if (m_fd == -1)
      throw std::runtime_error("shm_open failed");
    struct stat st;
    while (true) {
      fstat(m_fd, &st);
      if (st.st_size >= (off_t)sizeof(pthread_mutex_t))
        break;
      usleep(1000);
    }
  }

  m_mutex = static_cast<pthread_mutex_t *>(mmap(nullptr, sizeof(pthread_mutex_t), PROT_READ | PROT_WRITE, MAP_SHARED, m_fd, 0));
  if (m_mutex == MAP_FAILED) {
    m_mutex = nullptr;
    throw std::runtime_error("mmap failed");
  }

  if (initialize) {
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
    pthread_mutexattr_setrobust(&attr, PTHREAD_MUTEX_ROBUST);
    pthread_mutex_init(m_mutex, &attr);
    pthread_mutexattr_destroy(&attr);
  } else {
    int ret = pthread_mutex_trylock(m_mutex);
    if (ret == 0) {
      pthread_mutex_unlock(m_mutex);
    } else if (ret == EOWNERDEAD) {
      pthread_mutex_consistent(m_mutex);
      pthread_mutex_unlock(m_mutex);
    } else if (ret == ENOTRECOVERABLE) {
      throw std::runtime_error("Mutex is ENOTRECOVERABLE. Please delete /dev/shm" + shmName);
    } else if (ret != EBUSY) {
      throw std::runtime_error("Unexpected mutex state on attach: " + std::to_string(ret));
    }
  }
}

LinuxIpcMutex::~LinuxIpcMutex() {
  if (m_mutex && m_mutex != MAP_FAILED)
    munmap(m_mutex, sizeof(pthread_mutex_t));
  if (m_fd != -1)
    close(m_fd);
}

void LinuxIpcMutex::lock() {
  int ret = pthread_mutex_lock(m_mutex);
  if (ret == EOWNERDEAD) {
    pthread_mutex_consistent(m_mutex);
  } else if (ret != 0) {
    throw std::runtime_error("pthread_mutex_lock failed: " + std::to_string(ret));
  }
}

bool LinuxIpcMutex::try_lock() {
  int ret = pthread_mutex_trylock(m_mutex);
  if (ret == EOWNERDEAD) {
    pthread_mutex_consistent(m_mutex);
    return true;
  }
  return ret == 0;
}

void LinuxIpcMutex::unlock() { pthread_mutex_unlock(m_mutex); }

struct LinuxIpcEvent::EventData {
  std::atomic<uint32_t> state;
};

LinuxIpcEvent::LinuxIpcEvent(const char *name, bool manualReset) : m_name(name), m_fd(-1), m_data(nullptr), m_manualReset(manualReset) {
  std::string shmName = "/" + m_name + "_EVT_SHM";
  m_fd = shm_open(shmName.c_str(), O_CREAT | O_EXCL | O_RDWR, 0666);
  bool initialize = false;

  if (m_fd != -1) {
    initialize = true;
    if (ftruncate(m_fd, sizeof(EventData)) == -1)
      throw std::runtime_error("ftruncate failed");
  } else {
    m_fd = shm_open(shmName.c_str(), O_RDWR, 0666);
    if (m_fd == -1)
      throw std::runtime_error("shm_open failed");
    struct stat st;
    while (true) {
      fstat(m_fd, &st);
      if (st.st_size >= (off_t)sizeof(EventData))
        break;
      usleep(1000);
    }
  }

  m_data = static_cast<EventData *>(mmap(nullptr, sizeof(EventData), PROT_READ | PROT_WRITE, MAP_SHARED, m_fd, 0));
  if (m_data == MAP_FAILED) {
    m_data = nullptr;
    throw std::runtime_error("mmap failed");
  }

  if (initialize) {
    m_data->state.store(0, std::memory_order_relaxed);
  }
}

LinuxIpcEvent::~LinuxIpcEvent() {
  if (m_data && m_data != MAP_FAILED)
    munmap(m_data, sizeof(EventData));
  if (m_fd != -1)
    close(m_fd);
}

void LinuxIpcEvent::set() {
  if (m_manualReset) {
    // Manual reset: Set to 1 and wake EVERYONE waiting
    m_data->state.store(1, std::memory_order_release);
    syscall(SYS_futex, &m_data->state, FUTEX_WAKE, INT_MAX, nullptr, nullptr, 0);
  } else {
    // Auto reset: Set to 1, but only wake ONE waiting thread
    uint32_t expected = 0;
    if (m_data->state.compare_exchange_strong(expected, 1, std::memory_order_release)) {
      syscall(SYS_futex, &m_data->state, FUTEX_WAKE, 1, nullptr, nullptr, 0);
    }
  }
}

void LinuxIpcEvent::reset() { m_data->state.store(0, std::memory_order_release); }

bool LinuxIpcEvent::wait(uint32_t timeoutMs) {
  struct timespec end_time;
  bool has_timeout = (timeoutMs != 0xFFFFFFFF);

  if (has_timeout) {
    clock_gettime(CLOCK_MONOTONIC, &end_time);
    end_time.tv_sec += timeoutMs / 1000;
    end_time.tv_nsec += (timeoutMs % 1000) * 1000000;
    if (end_time.tv_nsec >= 1000000000) {
      end_time.tv_sec += 1;
      end_time.tv_nsec -= 1000000000;
    }
  }

  while (true) {
    if (m_manualReset) {
      // Manual reset just waits for state == 1
      if (m_data->state.load(std::memory_order_acquire) == 1)
        return true;
    } else {
      // Auto reset must claim the signal (atomically change 1 back to 0)
      uint32_t expected = 1;
      if (m_data->state.compare_exchange_strong(expected, 0, std::memory_order_acquire)) {
        return true;
      }
    }

    // Calculate remaining timeout
    struct timespec ts;
    struct timespec *ts_ptr = nullptr;
    if (has_timeout) {
      struct timespec now;
      clock_gettime(CLOCK_MONOTONIC, &now);
      ts.tv_sec = end_time.tv_sec - now.tv_sec;
      ts.tv_nsec = end_time.tv_nsec - now.tv_nsec;
      if (ts.tv_nsec < 0) {
        ts.tv_sec -= 1;
        ts.tv_nsec += 1000000000;
      }
      if (ts.tv_sec < 0) {
        return false; // Timed out
      }
      ts_ptr = &ts;
    }

    // Wait until state is no longer 0
    int ret = syscall(SYS_futex, &m_data->state, FUTEX_WAIT, 0, ts_ptr, nullptr, 0);
    if (ret == -1 && errno == ETIMEDOUT) {
      return false;
    }
  }
}

LinuxIpcSharedMemory::LinuxIpcSharedMemory(const char *name, size_t size) : m_name(name), m_fd(-1), m_size(size), m_ptr(nullptr) {
  std::string shmName = "/" + m_name;
  m_fd = shm_open(shmName.c_str(), O_CREAT | O_RDWR, 0666);
  if (m_fd == -1)
    throw std::runtime_error("shm_open failed");

  struct stat st;
  fstat(m_fd, &st);
  if (st.st_size < (off_t)size) {
    if (ftruncate(m_fd, size) == -1)
      throw std::runtime_error("ftruncate failed");
  }
}

LinuxIpcSharedMemory::~LinuxIpcSharedMemory() {
  unmap();
  if (m_fd != -1)
    close(m_fd);
}

void *LinuxIpcSharedMemory::map() {
  if (!m_ptr && m_fd != -1) {
    m_ptr = mmap(nullptr, m_size, PROT_READ | PROT_WRITE, MAP_SHARED, m_fd, 0);
    if (m_ptr == MAP_FAILED) {
      m_ptr = nullptr;
      throw std::runtime_error("mmap failed");
    }
  }
  return m_ptr;
}

void LinuxIpcSharedMemory::unmap() {
  if (m_ptr) {
    munmap(m_ptr, m_size);
    m_ptr = nullptr;
  }
}

struct LinuxIpcBroadcast::BroadcastData {
  std::atomic<uint32_t> futex_word;
};

LinuxIpcBroadcast::LinuxIpcBroadcast(const char *name) : m_name(name), m_fd(-1), m_data(nullptr) {
  std::string shmName = "/" + m_name + "_BCAST_SHM";
  m_fd = shm_open(shmName.c_str(), O_CREAT | O_EXCL | O_RDWR, 0666);
  bool initialize = false;
  if (m_fd != -1) {
    initialize = true;
    if (ftruncate(m_fd, sizeof(BroadcastData)) == -1)
      throw std::runtime_error("ftruncate failed");
  } else {
    m_fd = shm_open(shmName.c_str(), O_RDWR, 0666);
    if (m_fd == -1)
      throw std::runtime_error("shm_open failed");
    struct stat st;
    while (true) {
      fstat(m_fd, &st);
      if (st.st_size >= (off_t)sizeof(BroadcastData))
        break;
      usleep(1000);
    }
  }

  m_data = static_cast<BroadcastData *>(mmap(nullptr, sizeof(BroadcastData), PROT_READ | PROT_WRITE, MAP_SHARED, m_fd, 0));
  if (m_data == MAP_FAILED) {
    m_data = nullptr;
    throw std::runtime_error("mmap failed");
  }

  if (initialize) {
    m_data->futex_word.store(0, std::memory_order_relaxed);
  }
}

LinuxIpcBroadcast::~LinuxIpcBroadcast() {
  if (m_data && m_data != MAP_FAILED)
    munmap(m_data, sizeof(BroadcastData));
  if (m_fd != -1)
    close(m_fd);
}

bool LinuxIpcBroadcast::wait(uint32_t timeoutMs) {
  uint32_t current = m_data->futex_word.load(std::memory_order_acquire);

  struct timespec ts;
  struct timespec *ts_ptr = nullptr;
  if (timeoutMs != 0xFFFFFFFF) {
    ts.tv_sec = timeoutMs / 1000;
    ts.tv_nsec = (timeoutMs % 1000) * 1000000;
    ts_ptr = &ts;
  }

  syscall(SYS_futex, &m_data->futex_word, FUTEX_WAIT, current, ts_ptr, nullptr, 0);

  uint32_t currentAfterWait = m_data->futex_word.load(std::memory_order_acquire);
  if (current != currentAfterWait) {
    return true;
  }

  // We timed out
  return false;
}

void LinuxIpcBroadcast::notify_all() {
  m_data->futex_word.fetch_add(1, std::memory_order_release);
  syscall(SYS_futex, &m_data->futex_word, FUTEX_WAKE, INT_MAX, nullptr, nullptr, 0);
}

void *LinuxIpcMutex::get_native_handle() { return reinterpret_cast<void *>(static_cast<uintptr_t>(m_fd)); }

void *LinuxIpcEvent::get_native_handle() { return reinterpret_cast<void *>(static_cast<uintptr_t>(m_fd)); }

void *LinuxIpcSharedMemory::get_native_handle() { return reinterpret_cast<void *>(static_cast<uintptr_t>(m_fd)); }

#endif // __linux__