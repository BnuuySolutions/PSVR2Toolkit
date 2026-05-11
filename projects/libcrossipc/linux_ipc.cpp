#if defined(__linux__)

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


LinuxIpcMutex::LinuxIpcMutex(const char *name)
    : m_name(name), m_fd(-1), m_mutex(nullptr) {
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

  m_mutex = static_cast<pthread_mutex_t *>(
      mmap(nullptr, sizeof(pthread_mutex_t), PROT_READ | PROT_WRITE, MAP_SHARED, m_fd, 0));
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
    throw std::runtime_error("pthread_mutex_lock failed: " +
                             std::to_string(ret));
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
  pthread_mutex_t mutex;
  pthread_cond_t cond;
  bool signaled;
};

LinuxIpcEvent::LinuxIpcEvent(const char *name)
    : m_name(name), m_fd(-1), m_data(nullptr) {
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

  m_data = static_cast<EventData *>(mmap(
      nullptr, sizeof(EventData), PROT_READ | PROT_WRITE, MAP_SHARED, m_fd, 0));
  if (m_data == MAP_FAILED) {
    m_data = nullptr;
    throw std::runtime_error("mmap failed");
  }

  if (initialize) {
    pthread_mutexattr_t m_attr;
    pthread_mutexattr_init(&m_attr);
    pthread_mutexattr_setpshared(&m_attr, PTHREAD_PROCESS_SHARED);
    pthread_mutexattr_setrobust(&m_attr, PTHREAD_MUTEX_ROBUST);
    pthread_mutex_init(&m_data->mutex, &m_attr);
    pthread_mutexattr_destroy(&m_attr);

    pthread_condattr_t c_attr;
    pthread_condattr_init(&c_attr);
    pthread_condattr_setpshared(&c_attr, PTHREAD_PROCESS_SHARED);
    pthread_condattr_setclock(&c_attr, CLOCK_MONOTONIC);
    pthread_cond_init(&m_data->cond, &c_attr);
    pthread_condattr_destroy(&c_attr);

    m_data->signaled = false;
  }
}

LinuxIpcEvent::~LinuxIpcEvent() {
  if (m_data && m_data != MAP_FAILED)
    munmap(m_data, sizeof(EventData));
  if (m_fd != -1)
    close(m_fd);
}

void LinuxIpcEvent::set() {
  int ret = pthread_mutex_lock(&m_data->mutex);
  if (ret == EOWNERDEAD)
    pthread_mutex_consistent(&m_data->mutex);

  m_data->signaled = true;
  pthread_cond_signal(&m_data->cond);

  pthread_mutex_unlock(&m_data->mutex);
}

bool LinuxIpcEvent::wait(uint32_t timeoutMs) {
  int ret = pthread_mutex_lock(&m_data->mutex);
  if (ret == EOWNERDEAD)
    pthread_mutex_consistent(&m_data->mutex);

  bool success = true;
  if (!m_data->signaled) {
    if (timeoutMs == 0xFFFFFFFF) {
      while (!m_data->signaled) {
        pthread_cond_wait(&m_data->cond, &m_data->mutex);
      }
    } else {
      struct timespec ts;
      clock_gettime(CLOCK_MONOTONIC, &ts);
      ts.tv_sec += timeoutMs / 1000;
      ts.tv_nsec += (timeoutMs % 1000) * 1000000;
      if (ts.tv_nsec >= 1000000000) {
        ts.tv_sec += 1;
        ts.tv_nsec -= 1000000000;
      }

      while (!m_data->signaled) {
        int err = pthread_cond_timedwait(&m_data->cond, &m_data->mutex, &ts);
        if (err == ETIMEDOUT) {
          if (!m_data->signaled)
            success = false;
          break;
        }
      }
    }
  }

  if (success) {
    m_data->signaled = false; // Auto-reset behavior
  }

  pthread_mutex_unlock(&m_data->mutex);
  return success;
}

LinuxIpcSharedMemory::LinuxIpcSharedMemory(const char *name, size_t size)
    : m_name(name), m_fd(-1), m_size(size), m_ptr(nullptr) {
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

LinuxIpcBroadcast::LinuxIpcBroadcast(const char *name)
    : m_name(name), m_fd(-1), m_data(nullptr), m_last_val(0) {
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

  m_data = static_cast<BroadcastData *>(mmap(nullptr, sizeof(BroadcastData),
                                             PROT_READ | PROT_WRITE, MAP_SHARED,
                                             m_fd, 0));
  if (m_data == MAP_FAILED) {
    m_data = nullptr;
    throw std::runtime_error("mmap failed");
  }

  if (initialize) {
    m_data->futex_word.store(0, std::memory_order_relaxed);
  }

  m_last_val = m_data->futex_word.load(std::memory_order_acquire);
}

LinuxIpcBroadcast::~LinuxIpcBroadcast() {
  if (m_data && m_data != MAP_FAILED)
    munmap(m_data, sizeof(BroadcastData));
  if (m_fd != -1)
    close(m_fd);
}

bool LinuxIpcBroadcast::wait(uint32_t timeoutMs) {
  uint32_t current = m_data->futex_word.load(std::memory_order_acquire);
  if (current != m_last_val) {
    m_last_val = current;
    return true;
  }

  struct timespec ts;
  struct timespec *ts_ptr = nullptr;
  if (timeoutMs != 0xFFFFFFFF) {
    ts.tv_sec = timeoutMs / 1000;
    ts.tv_nsec = (timeoutMs % 1000) * 1000000;
    ts_ptr = &ts;
  }

  syscall(SYS_futex, &m_data->futex_word, FUTEX_WAIT, current, ts_ptr, nullptr, 0);

  current = m_data->futex_word.load(std::memory_order_acquire);
  if (current != m_last_val) {
    m_last_val = current;
    return true;
  }
  return false;
}

void LinuxIpcBroadcast::notify_all() {
  m_data->futex_word.fetch_add(1, std::memory_order_release);
  syscall(SYS_futex, &m_data->futex_word, FUTEX_WAKE, INT_MAX, nullptr, nullptr, 0);
}

#endif