#pragma once

#include <mutex>

namespace psvr2_toolkit {
  struct LibpadTimeSync
  {
    uint8_t unknown0x0[0x810];
    uint8_t isLeft;
    uint8_t unknown0x812[0x16f];
  }; static_assert(sizeof(LibpadTimeSync) == 0x980, "Size of LibpadTimeSync is not 0x980 bytes!");

  enum LedPhase : uint8_t {
      INIT = 0x0,
      PRESCAN = 0x1,
      BROAD = 0x2,
      BG = 0x3,
      STABLE = 0x4,
      LED_ALL_OFF = 0x5,
      LED_ALL_ON = 0x6,
      DEBUG = 0x7
  };

  // This is slightly different from SenseLED_t, as this
  // is not packed and contains some additional data.
  struct LibpadLedSync
  {
    LedPhase phase;
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
  }; static_assert(sizeof(LibpadLedSync) == 0x30, "Size of LibpadLedSync is not 0x30 bytes!");

  class LibpadHooks {
  public:
    static void InstallHooks();
  };

} // psvr2_toolkit
