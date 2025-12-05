#pragma once

#include "hmd_types.h"

#include <mutex>

namespace psvr2_toolkit {
  extern std::mutex ldPayloadMutex;
  extern LDPayload currentLDPayload;

  class UsbThreadHooks {
  public:
    static void InstallHooks();
  };

} // psvr2_toolkit
