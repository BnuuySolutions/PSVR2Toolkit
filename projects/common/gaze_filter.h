#pragma once

// This file i'll be using the following filters:
//   * a one-euro filter (Casiez, Roussel & Vogel, 2012) whose cutoff rises with gaze velocity, and
//   * an I-VT classifier that detects saccades and passes them through untouched.
//
// It also holds the last good direction briefly across blinks, where the tracker drops out for ~100-150
// ms and would otherwise make a gaze cursor fall to the floor and snap back
//
// Deliberately free of SteamVR, Windows and driver dependencies so it can be exercised off-device;
// see psvr2_gaze_filter_test.

#include <cmath>
#include <cstdint>

struct GazeFilterVec3 {
  float x, y, z;
};

enum class GazeMotionState : uint8_t {
  Unknown = 0,
  Fixation = 1,
  Saccade = 2,
};

struct GazeFilterConfig {
  // One-euro: the cutoff applied when gaze is still. Lower is smoother and laggier.
  float minCutoffHz = 1.0f;
  // IF YOU DON'T KNOW WHA'S GOING ON, READ THE ONE-EURO PAPER. This is the "beta" parameter that controls how much the cutoff rises with velocity
  // The canonical one-euro beta is dimensioned by whatever units the input carries. Gaze directions are
  // unit vectors, so their derivative is in rad/s -- only ~0.44 at 25 deg/s. Feeding that to the
  // literature's beta of 0.007 moves the cutoff by 0.003 Hz, i.e. no adaptation whatsoever. Working in
  // deg/s instead makes this value interpretable and actually effective.

  // I recommend reading about the 1 euro formula btw, it's a very good filter and I have been using it on my tracking and math projects :pppp
  float beta = 0.12f;
  // One-euro: cutoff for the velocity estimate itself.
  float derivativeCutoffHz = 1.0f;
  // I-VT threshold.
  //
  // Must sit above the tracker's noise floor, not just above normal eye drift: +/-0.3 deg of per-sample
  // noise at 120 Hz already looks like ~70 deg/s. A threshold down at the 30 deg/s often quoted for
  // clean lab data would classify ordinary fixation noise as saccades and bypass filtering entirely.
  // Real saccades of any size run well into the hundreds of deg/s.
  float saccadeVelocityDegPerSec = 120.0f;
  // How long to keep reporting the last good direction after validity drops.
  int64_t blinkHoldUs = 200000;
  // Beyond this gap the filter history is stale and gets discarded rather than smoothed against.
  int64_t maxSampleGapUs = 500000;
};

struct GazeFilterOutput {
  GazeFilterVec3 direction{0.0f, 0.0f, 0.0f};
  GazeMotionState state = GazeMotionState::Unknown;
  // Whether a usable direction is being reported at all.
  bool valid = false;
  // True when `direction` is a held sample carried across a dropout rather than a fresh measurement.
  bool held = false;
};

class GazeFilter {
public:
  void Configure(const GazeFilterConfig &config) { m_config = config; }
  const GazeFilterConfig &Config() const { return m_config; }

  void Reset() {
    m_hasPrevious = false;
    m_hasLastValid = false;
    m_state = GazeMotionState::Unknown;
  }

  GazeFilterOutput Update(const GazeFilterVec3 &direction, bool inputValid, int64_t timestampUs) {
    GazeFilterOutput out;

    if (!inputValid) {
      return HoldOrDrop(timestampUs);
    }

    // normalise to {0,0,0}, which is finite and would sail through as if it were a real measurement
    const float inputLength = Length(direction);
    if (!std::isfinite(inputLength) || inputLength <= 1e-6f) {
      return HoldOrDrop(timestampUs);
    }

    const GazeFilterVec3 normalized{direction.x / inputLength, direction.y / inputLength, direction.z / inputLength};

    // First usable sample, or the history is too old to smooth against.
    const int64_t deltaUs = m_hasPrevious ? (timestampUs - m_previousTimestampUs) : 0;
    if (!m_hasPrevious || deltaUs <= 0 || deltaUs > m_config.maxSampleGapUs) {
      SeedFrom(normalized, timestampUs);

      out.direction = normalized;
      out.state = GazeMotionState::Unknown;
      out.valid = true;
      out.held = false;
      return out;
    }

    const float deltaSeconds = static_cast<float>(deltaUs) / 1e6f;

    // Angular velocity between consecutive raw samples drives both the classifier and the one-euro
    // cutoff. Using the raw input rather than the filtered output keeps the filter from hiding the
    // very motion it needs to react to.
    const float velocityDegPerSec = AngleBetweenDegrees(m_previousRaw, normalized) / deltaSeconds;

    m_previousRaw = normalized;
    m_previousTimestampUs = timestampUs;

    if (velocityDegPerSec >= m_config.saccadeVelocityDegPerSec) {
      // Saccade: report the measurement as-is and restart smoothing from it. Filtering here would add
      // latency exactly where the eye is moving fastest and the user notices most.
      m_state = GazeMotionState::Saccade;
      m_filtered = normalized;
      m_derivative = GazeFilterVec3{0.0f, 0.0f, 0.0f};
      m_lastValid = normalized;
      m_lastValidTimestampUs = timestampUs;
      m_hasLastValid = true;

      out.direction = normalized;
      out.state = GazeMotionState::Saccade;
      out.valid = true;
      out.held = false;
      return out;
    }

    m_state = GazeMotionState::Fixation;

    // One-euro here.
    const GazeFilterVec3 rawDerivative{(normalized.x - m_filtered.x) / deltaSeconds, (normalized.y - m_filtered.y) / deltaSeconds,
                                       (normalized.z - m_filtered.z) / deltaSeconds};

    const float derivativeAlpha = SmoothingFactor(m_config.derivativeCutoffHz, deltaSeconds);
    m_derivative = Lerp(m_derivative, rawDerivative, derivativeAlpha);

    // For a unit-length direction the derivative magnitude is angular velocity in rad/s; beta is
    // expressed per deg/s, so convert. Note this uses the LOW-PASSED derivative, not the raw derivative: the one-euro paper explicitly warns that raw derivative
    // noise averages towards zero here, which is what keeps the cutoff low while the eye is still
    const float derivativeDegPerSec = Length(m_derivative) * (180.0f / 3.14159265358979323846f);
    const float cutoff = m_config.minCutoffHz + m_config.beta * derivativeDegPerSec;
    const float alpha = SmoothingFactor(cutoff, deltaSeconds);

    m_filtered = Normalize(Lerp(m_filtered, normalized, alpha));

    m_lastValid = m_filtered;
    m_lastValidTimestampUs = timestampUs;
    m_hasLastValid = true;

    out.direction = m_filtered;
    out.state = GazeMotionState::Fixation;
    out.valid = true;
    out.held = false;
    return out;
  }

private:
  void SeedFrom(const GazeFilterVec3 &direction, int64_t timestampUs) {
    m_filtered = direction;
    m_previousRaw = direction;
    m_derivative = GazeFilterVec3{0.0f, 0.0f, 0.0f};
    m_previousTimestampUs = timestampUs;
    m_hasPrevious = true;

    m_lastValid = direction;
    m_lastValidTimestampUs = timestampUs;
    m_hasLastValid = true;
    m_state = GazeMotionState::Unknown;
  }

  GazeFilterOutput HoldOrDrop(int64_t timestampUs) {
    GazeFilterOutput out;

    if (!m_hasLastValid) {
      return out;
    }

    const int64_t heldForUs = timestampUs - m_lastValidTimestampUs;
    if (heldForUs < 0 || heldForUs > m_config.blinkHoldUs) {
      m_hasPrevious = false;
      m_hasLastValid = false;
      m_state = GazeMotionState::Unknown;
      return out;
    }

  
    m_hasPrevious = false;

    out.direction = m_lastValid;
    out.state = m_state;
    out.valid = true;
    out.held = true;
    return out;
  }

  static float SmoothingFactor(float cutoffHz, float deltaSeconds) {
    if (cutoffHz <= 0.0f) {
      return 1.0f;
    }
    const float tau = 1.0f / (6.2831853f * cutoffHz);
    return 1.0f / (1.0f + tau / deltaSeconds);
  }

  static GazeFilterVec3 Lerp(const GazeFilterVec3 &from, const GazeFilterVec3 &to, float alpha) {
    return GazeFilterVec3{from.x + (to.x - from.x) * alpha, from.y + (to.y - from.y) * alpha, from.z + (to.z - from.z) * alpha};
  }

  static float Length(const GazeFilterVec3 &v) { return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }

  static bool IsFinite(const GazeFilterVec3 &v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

  static GazeFilterVec3 Normalize(const GazeFilterVec3 &v) {
    const float length = Length(v);
    if (length <= 1e-6f || !std::isfinite(length)) {
      return GazeFilterVec3{0.0f, 0.0f, 0.0f};
    }
    return GazeFilterVec3{v.x / length, v.y / length, v.z / length};
  }

  static float AngleBetweenDegrees(const GazeFilterVec3 &a, const GazeFilterVec3 &b) {
    float dot = a.x * b.x + a.y * b.y + a.z * b.z;
    if (dot > 1.0f) {
      dot = 1.0f;
    } else if (dot < -1.0f) {
      dot = -1.0f;
    }
    return std::acos(dot) * (180.0f / 3.14159265358979323846f);
  }

  GazeFilterConfig m_config;

  GazeFilterVec3 m_filtered{0.0f, 0.0f, 0.0f};
  GazeFilterVec3 m_previousRaw{0.0f, 0.0f, 0.0f};
  GazeFilterVec3 m_derivative{0.0f, 0.0f, 0.0f};
  int64_t m_previousTimestampUs = 0;
  bool m_hasPrevious = false;

  GazeFilterVec3 m_lastValid{0.0f, 0.0f, 0.0f};
  int64_t m_lastValidTimestampUs = 0;
  bool m_hasLastValid = false;

  GazeMotionState m_state = GazeMotionState::Unknown;
};
