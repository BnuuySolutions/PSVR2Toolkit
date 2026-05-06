#pragma once

#include <atomic>
#include <thread>

namespace psvr2_toolkit {
  class CommandThread {
  public:
    static void Initialize();
    static void Stop();

  private:
    static void ThreadLoop();

    static std::atomic<bool> m_running;
    static std::thread m_thread;
  };
}