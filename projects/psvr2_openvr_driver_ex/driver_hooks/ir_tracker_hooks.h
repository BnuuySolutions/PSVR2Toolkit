#pragma once

#include <atomic>
#include <cstdint>

namespace psvr2_toolkit {

class IrTrackerHooks {
public:
  static std::atomic<int32_t> g_controllerLedCount[2];
  static std::atomic<uint64_t> g_opticalFrameIndex[2];

  static void InstallHooks();
};

} // namespace psvr2_toolkit
