#pragma once

#include "rolling_percentile.h"
#include "sense_controller.h"
#include <mutex>

namespace psvr2_toolkit {
  struct TimeSync
  {
    uint8_t unknown0x0[0x810];
    uint8_t isLeft;
    uint8_t unknown0x812[0x16f];
  }; static_assert(sizeof(TimeSync) == 0x980, "Size of TimeSync is not 0x980 bytes!");

  // This is slightly different from SenseLED_t, as this
  // is not packed and contains some additional data.
  struct LedSync
  {
    SenseLEDPhase phase;
    uint8_t seq;
    uint8_t period;
    int32_t baseTime;
    int32_t frameCycle;
    uint8_t leds[4];
    int64_t last_timestamp;
    bool unknown0x18; // Is LED sync data valid?
    int32_t camExposure;
    int32_t oneSubGridTime;
    int32_t oneSubGridTime_x15; // Unsure what this is, but this is oneSubGridTime times 15.
    bool unknown0x28; // Is camera data valid?
  }; static_assert(sizeof(LedSync) == 0x30, "Size of LedSync is not 0x30 bytes!");

  // TODO: Move this into SenseController
  struct ControllerStruct
  {
    RollingPercentile<double> timeStampOffset = RollingPercentile<double>(5000, 99.9);
    bool isTracking = false;
	uint64_t lastTrackedTimestamp = 0;
    TimeSync* timeSync;
    LedSync* ledSync;
    std::mutex mutex;
  };

  class LibpadHooks {
  public:
    static void InstallHooks();
  };

} // psvr2_toolkit
