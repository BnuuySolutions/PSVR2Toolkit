#pragma once

#include "rolling_percentile.h"
#include <mutex>

namespace psvr2_toolkit {
  struct TimeStruct
  {
    RollingPercentile<double> timeStampOffset = RollingPercentile<double>(5000, 99.9);
    bool shouldBumpLEDSequence = false;
    std::mutex mutex;
  };

  class LibpadHooks {
  public:
    static void InstallHooks();
  };

} // psvr2_toolkit
