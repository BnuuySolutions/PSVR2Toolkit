#pragma once

#include "rolling_percentile.h"
#include <mutex>

namespace psvr2_toolkit {
  // TODO: Move this into SenseController
  struct ControllerStruct
  {
    RollingPercentile<double> timeStampOffset = RollingPercentile<double>(5000, 99.9);
    bool isTracking = false;
	uint64_t lastTrackedTimestamp = 0;
    std::mutex mutex;
  };

  class LibpadHooks {
  public:
    static void InstallHooks();
  };

} // psvr2_toolkit
