// Off-device tests for GazeFilter (common/gaze_filter.h).
//
// The point of this filter is to settle fixations WITHOUT adding lag to saccades, so the tests measure
// both properties rather than just checking it runs. See EYE_TRACKING_PLAN.md 5.1.

#include "gaze_filter.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace {

int g_failures = 0;

void Check(bool condition, const std::string &name) {
  if (condition) {
    std::printf("  pass  %s\n", name.c_str());
  } else {
    std::printf("  FAIL  %s\n", name.c_str());
    ++g_failures;
  }
}

void CheckNear(float actual, float expected, float tolerance, const std::string &name) {
  const bool ok = std::fabs(actual - expected) <= tolerance;
  if (ok) {
    std::printf("  pass  %s (%.4f)\n", name.c_str(), actual);
  } else {
    std::printf("  FAIL  %s: got %.4f, expected %.4f +/- %.4f\n", name.c_str(), actual, expected, tolerance);
    ++g_failures;
  }
}

constexpr int64_t k_frameIntervalUs = 8333; // ~120 Hz

// A direction `yawDeg` off straight ahead, in the horizontal plane.
GazeFilterVec3 DirectionAtYaw(float yawDeg) {
  const float rad = yawDeg * (3.14159265358979323846f / 180.0f);
  return GazeFilterVec3{std::sin(rad), 0.0f, -std::cos(rad)};
}

float AngleBetweenDeg(const GazeFilterVec3 &a, const GazeFilterVec3 &b) {
  float dot = a.x * b.x + a.y * b.y + a.z * b.z;
  dot = dot > 1.0f ? 1.0f : (dot < -1.0f ? -1.0f : dot);
  return std::acos(dot) * (180.0f / 3.14159265358979323846f);
}

// Deterministic pseudo-noise so the test does not depend on a random engine's implementation.
struct Jitter {
  uint32_t state = 0x12345678u;
  float Next(float amplitudeDeg) {
    state = state * 1664525u + 1013904223u;
    const float unit = static_cast<float>((state >> 8) & 0xFFFF) / 65535.0f; // [0,1]
    return (unit * 2.0f - 1.0f) * amplitudeDeg;
  }
};

} // namespace

int main() {
  std::printf("GazeFilter\n");

  // --- Basic behaviour -----------------------------------------------------------------------

  {
    GazeFilter filter;
    const GazeFilterOutput out = filter.Update(DirectionAtYaw(0.0f), true, 1000);
    Check(out.valid && !out.held, "first sample is reported immediately");
    CheckNear(AngleBetweenDeg(out.direction, DirectionAtYaw(0.0f)), 0.0f, 0.001f, "first sample passes through unchanged");
  }

  {
    GazeFilter filter;
    const GazeFilterOutput out = filter.Update(DirectionAtYaw(0.0f), false, 1000);
    Check(!out.valid, "invalid input with no history reports invalid");
  }

  {
    GazeFilter filter;
    int64_t t = 0;
    for (int i = 0; i < 50; ++i) {
      t += k_frameIntervalUs;
      filter.Update(DirectionAtYaw(0.0f), true, t);
    }
    const GazeFilterOutput out = filter.Update(DirectionAtYaw(0.0f), true, t + k_frameIntervalUs);
    CheckNear(AngleBetweenDeg(out.direction, DirectionAtYaw(0.0f)), 0.0f, 0.01f, "a perfectly steady input is not dragged off target");
    Check(out.state == GazeMotionState::Fixation, "steady input classifies as fixation");
  }

  // --- Fixation: the filter must actually remove jitter --------------------------------------

  {
    GazeFilter filter;
    Jitter noiseSource;
    Jitter referenceSource;

    int64_t t = 0;
    double filteredError = 0.0;
    double rawError = 0.0;
    int samples = 0;

    // Warm up first so the comparison is of steady-state behaviour.
    for (int i = 0; i < 30; ++i) {
      t += k_frameIntervalUs;
      filter.Update(DirectionAtYaw(noiseSource.Next(0.3f)), true, t);
      referenceSource.Next(0.3f);
    }

    for (int i = 0; i < 200; ++i) {
      t += k_frameIntervalUs;
      const float noiseDeg = noiseSource.Next(0.3f);
      const GazeFilterOutput out = filter.Update(DirectionAtYaw(noiseDeg), true, t);

      filteredError += AngleBetweenDeg(out.direction, DirectionAtYaw(0.0f));
      rawError += std::fabs(noiseDeg);
      ++samples;
    }

    const double filteredMean = filteredError / samples;
    const double rawMean = rawError / samples;

    std::printf("        fixation: raw mean error %.4f deg, filtered %.4f deg\n", rawMean, filteredMean);
    Check(filteredMean < rawMean * 0.5, "fixation jitter is at least halved");
  }

  // --- Saccades: the filter must NOT add lag --------------------------------------------------

  {
    GazeFilter filter;
    int64_t t = 0;

    for (int i = 0; i < 30; ++i) {
      t += k_frameIntervalUs;
      filter.Update(DirectionAtYaw(0.0f), true, t);
    }

    // A 20 degree jump in one frame is ~2400 deg/s, unambiguously a saccade.
    t += k_frameIntervalUs;
    const GazeFilterOutput out = filter.Update(DirectionAtYaw(20.0f), true, t);

    Check(out.state == GazeMotionState::Saccade, "a fast jump classifies as a saccade");
    CheckNear(AngleBetweenDeg(out.direction, DirectionAtYaw(20.0f)), 0.0f, 0.001f, "saccade target is reached in one frame (no added lag)");
  }

  {
    // Slow drift must NOT be treated as a saccade: 5 deg/s is well under the threshold.
    GazeFilter filter;
    int64_t t = 0;
    float yaw = 0.0f;
    GazeMotionState state = GazeMotionState::Unknown;

    for (int i = 0; i < 60; ++i) {
      t += k_frameIntervalUs;
      yaw += 5.0f * (static_cast<float>(k_frameIntervalUs) / 1e6f);
      state = filter.Update(DirectionAtYaw(yaw), true, t).state;
    }

    Check(state == GazeMotionState::Fixation, "slow drift is not misclassified as a saccade");
  }

  {
    // Smooth pursuit at 25 deg/s stays under the default 30 deg/s threshold and should track closely
    // rather than lagging far behind.
    GazeFilter filter;
    int64_t t = 0;
    float yaw = 0.0f;
    GazeFilterOutput out;

    for (int i = 0; i < 120; ++i) {
      t += k_frameIntervalUs;
      yaw += 25.0f * (static_cast<float>(k_frameIntervalUs) / 1e6f);
      out = filter.Update(DirectionAtYaw(yaw), true, t);
    }

    const float lagDeg = AngleBetweenDeg(out.direction, DirectionAtYaw(yaw));
    std::printf("        pursuit: steady-state lag %.4f deg\n", lagDeg);
    Check(lagDeg < 3.0f, "smooth pursuit lag stays under 3 degrees");
  }

  // --- Blink hold -----------------------------------------------------------------------------

  {
    GazeFilter filter;
    int64_t t = 0;
    for (int i = 0; i < 30; ++i) {
      t += k_frameIntervalUs;
      filter.Update(DirectionAtYaw(10.0f), true, t);
    }

    // 100 ms of dropout, inside the default 200 ms hold.
    t += 100000;
    const GazeFilterOutput held = filter.Update(GazeFilterVec3{0, 0, 0}, false, t);
    Check(held.valid && held.held, "direction is held through a short dropout");
    CheckNear(AngleBetweenDeg(held.direction, DirectionAtYaw(10.0f)), 0.0f, 0.5f, "held direction is the last good one");

    // 300 ms total, past the hold window.
    t += 200000;
    const GazeFilterOutput dropped = filter.Update(GazeFilterVec3{0, 0, 0}, false, t);
    Check(!dropped.valid, "past the hold window the filter reports invalid");
  }

  {
    // Recovery after a long gap must not smooth the new sample against stale history.
    GazeFilter filter;
    int64_t t = 0;
    for (int i = 0; i < 30; ++i) {
      t += k_frameIntervalUs;
      filter.Update(DirectionAtYaw(0.0f), true, t);
    }

    t += 5000000; // 5 s
    const GazeFilterOutput out = filter.Update(DirectionAtYaw(30.0f), true, t);
    CheckNear(AngleBetweenDeg(out.direction, DirectionAtYaw(30.0f)), 0.0f, 0.001f, "sample after a long gap is not blended with stale history");
  }

  // --- Robustness ------------------------------------------------------------------------------

  {
    GazeFilter filter;
    const GazeFilterOutput out = filter.Update(GazeFilterVec3{0.0f, 0.0f, 0.0f}, true, 1000);
    Check(!out.valid, "a zero-length direction is rejected");
  }

  {
    GazeFilter filter;
    const float nan = std::nanf("");
    const GazeFilterOutput out = filter.Update(GazeFilterVec3{nan, nan, nan}, true, 1000);
    Check(!out.valid, "a non-finite direction is rejected");
  }

  {
    // Non-monotonic timestamps must not blow up or produce garbage.
    GazeFilter filter;
    filter.Update(DirectionAtYaw(0.0f), true, 100000);
    const GazeFilterOutput out = filter.Update(DirectionAtYaw(1.0f), true, 50000);
    Check(out.valid, "a backwards timestamp is handled without producing an invalid sample");
  }

  {
    // Output must stay unit length, since it becomes a ray direction downstream.
    GazeFilter filter;
    Jitter noiseSource;
    int64_t t = 0;
    float worstDeviation = 0.0f;

    for (int i = 0; i < 200; ++i) {
      t += k_frameIntervalUs;
      const GazeFilterOutput out = filter.Update(DirectionAtYaw(noiseSource.Next(5.0f)), true, t);
      if (out.valid) {
        const float length = std::sqrt(out.direction.x * out.direction.x + out.direction.y * out.direction.y + out.direction.z * out.direction.z);
        worstDeviation = std::fmax(worstDeviation, std::fabs(length - 1.0f));
      }
    }

    CheckNear(worstDeviation, 0.0f, 0.001f, "output stays normalised");
  }

  std::printf("\n%s (%d failure%s)\n", g_failures == 0 ? "PASSED" : "FAILED", g_failures, g_failures == 1 ? "" : "s");
  return g_failures == 0 ? 0 : 1;
}
