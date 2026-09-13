#pragma once

// Pupillometry
//
// The headset reports pupil diameter per eye at 120 Hz. Delivering that number to an application directly would be misleading: it is dominated by the pupillary light reflex, contaminated by blink artifacts
// of the same magnitude as the effect people actually want, and biased by the angle the eye is turned
// relative to the camera. This runs the corrections that make it mean something.
//
// Stages, in order (at least of what I've worked with so far):
//
//   1. Validity and blink gating, with a guard band either side of every blink edge.
//   2. Foreshortening correction against a per-eye camera axis.
//   3. Binocular fusion when both eyes are usable.
//   4. Low-pass filtering into the band where pupil responses actually live.
//   5. Rolling baseline, with subtractive and divisive normalisation.
//   6. A luminance-robust activity index.
//

// WHAT THIS CANNOT DO!!!! 
// There is no display-luminance signal available anywhere in the headset
// telemetry -- exp_l/exp_r and led_status are constants describing the IR illuminator, not the screen.
// So absolute diameter can never be separated from "the scene got brighter". `activityIndex` exists
// precisely because it measures the RATE of abrupt dilations rather than their absolute level, which
// is far less sensitive to a slow luminance ramp. Treat `deltaMm`/`relative` as descriptive, and do
// not present either as a load measurement.
//
// Calibrate the camera axes with PupilCameraAxisCalibrator before trusting stage 2; the defaults are a
// starting point, not a fit for your headset.
//
// Dependency-free so it can be exercised off-device; see psvr2_pupillometry_test.
//Update made by LukeTheProtogen :PPP

#include <algorithm> // std::nth_element
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

struct PupilEyeSample {
  float diameterMm = 0.0f;
  bool diameterValid = false;
  bool blinking = false;


  float gazeDirX = 0.0f;
  float gazeDirY = 0.0f;
  float gazeDirZ = 0.0f;
  bool gazeValid = false;
};

struct PupilFrame {
  int64_t timestampUs = 0;
  PupilEyeSample left;
  PupilEyeSample right;
};

struct PupillometryConfig {
  float leftCameraAzimuthDeg = 0.0f;
  float leftCameraElevationDeg = 0.0f;
  float rightCameraAzimuthDeg = 0.0f;
  float rightCameraElevationDeg = 0.0f;
  // amplifies noise faster than it removes bias
  float minCorrectionCosine = 0.5f;

  // Samples within this of a blink edge are discarded, on both sides. Measured blink artifacts on real
  // data reached 0.27 mm, which is the size of the entire effect being looked for -- this is the single
  // most important stage here, and it is why results are delayed (see Update).
  int64_t blinkGuardUs = 100000;

  // Rolling baseline length. Long enough to average out the light reflex, short enough to track drift.
  float baselineWindowSeconds = 20.0f;

  // Pupil responses live below ~2 Hz; anything above this is sensor noise.
  float lowPassHz = 4.0f;

  // Window the activity index is computed over.
  float activityWindowSeconds = 4.0f;

  // Plausibility gate on raw diameter, in millimetres.
  float minPlausibleMm = 1.0f;
  float maxPlausibleMm = 9.0f;
};

struct PupillometryResult {
  int64_t timestampUs = 0;

  // True when this carries an artifact-free, corrected measurement.
  bool valid = false;

  float correctedMm = 0.0f; // foreshortening-corrected, binocular where both eyes were usable
  float smoothedMm = 0.0f;  // after the low-pass
  float baselineMm = 0.0f;
  float deltaMm = 0.0f;  // smoothed - baseline
  float relative = 0.0f; // (smoothed - baseline) / baseline

  // Abrupt dilations per second. Luminance-robust; this is the number to prefer.
  float activityIndex = 0.0f;

  bool baselineReady = false;
  bool activityReady = false;
  int contributingEyes = 0; // 0, 1 or 2
};

namespace pupillometry_detail {

// Builds a unit axis from azimuth/elevation, in the headset's +Z-forward convention.
inline void AxisFromAngles(float azimuthDeg, float elevationDeg, float &x, float &y, float &z) {
  const float az = azimuthDeg * (3.14159265358979323846f / 180.0f);
  const float el = elevationDeg * (3.14159265358979323846f / 180.0f);
  x = std::sin(az) * std::cos(el);
  y = std::sin(el);
  z = std::cos(az) * std::cos(el);
}

// Cosine of the angle between the gaze direction and the camera axis, 1.0 when the eye looks straight
// down the camera. Both vectors point away from the eye in the same convention, so this is a plain dot
// product -- no negation.
inline float CosineToAxis(const PupilEyeSample &eye, float ax, float ay, float az) {
  const float n = std::sqrt(eye.gazeDirX * eye.gazeDirX + eye.gazeDirY * eye.gazeDirY + eye.gazeDirZ * eye.gazeDirZ);
  if (n < 1e-6f || !std::isfinite(n)) {
    return 0.0f;
  }
  const float c = (eye.gazeDirX / n) * ax + (eye.gazeDirY / n) * ay + (eye.gazeDirZ / n) * az;
  return c > 1.0f ? 1.0f : (c < -1.0f ? -1.0f : c);
}

inline float Median(std::vector<float> values) {
  if (values.empty()) {
    return 0.0f;
  }
  const size_t mid = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + mid, values.end());
  return values[mid];
}

} // namespace pupillometry_detail

class PupillometryProcessor {
public:
  void Configure(const PupillometryConfig &config) { m_config = config; }
  const PupillometryConfig &Config() const { return m_config; }

  void Reset() {
    m_pending.clear();
    m_baseline.clear();
    m_activity.clear();
    m_haveSmoothed = false;
    m_lastEmitUs = 0;
    m_lastBlinkUs = 0;
  }

  // Feeds one frame and reports whether a processed result is ready.
  // Results lag the input by blinkGuardUs, because a sample can only be cleared once it is known that
  // no blink starts shortly after it. That delay is the price of not emitting blink artifacts, and at
  // the default 100 ms it is irrelevant for anything pupil responses are used for.
  bool Update(const PupilFrame &frame, PupillometryResult &out) {
    if (frame.left.blinking || frame.right.blinking) {
      m_lastBlinkUs = frame.timestampUs;
    }

    m_pending.push_back(frame);

    // Keep the buffer bounded even if timestamps misbehave.
    while (m_pending.size() > k_maxPending) {
      m_pending.pop_front();
    }

    const int64_t now = frame.timestampUs;
    if (m_pending.empty() || (now - m_pending.front().timestampUs) < m_config.blinkGuardUs) {
      return false;
    }

    const PupilFrame candidate = m_pending.front();
    m_pending.pop_front();

    // Non-monotonic input invalidates the windows we accumulate.
    if (m_lastEmitUs != 0 && candidate.timestampUs <= m_lastEmitUs) {
      return false;
    }

    out = Process(candidate, now);
    m_lastEmitUs = candidate.timestampUs;
    return true;
  }

private:
  static constexpr size_t k_maxPending = 512;

  // A sample is usable only if it is far enough from any blink edge on BOTH sides. Looking forward alone is not enough. The lid retracting after a blink partially occludes the pupil
  // for a while, and that recovery artifact is at least as large as the approach to the blink measured 0.27 mm swing spans both. Forward coverage comes from the pending buffer; backwardd coverage needs the timestamp of the last blink seen.
  bool NearBlink(int64_t timestampUs) const {
    if (m_lastBlinkUs != 0 && timestampUs > m_lastBlinkUs && (timestampUs - m_lastBlinkUs) <= m_config.blinkGuardUs) {
      return true;
    }

    for (const PupilFrame &f : m_pending) {
      if (!f.left.blinking && !f.right.blinking) {
        continue;
      }
      const int64_t dt = f.timestampUs - timestampUs;
      if (dt >= 0 && dt <= m_config.blinkGuardUs) {
        return true;
      }
    }
    return false;
  }

  bool CorrectEye(const PupilEyeSample &eye, float azimuthDeg, float elevationDeg, float &outMm) const {
    if (!eye.diameterValid || eye.blinking || !eye.gazeValid) {
      return false;
    }
    if (!std::isfinite(eye.diameterMm) || eye.diameterMm < m_config.minPlausibleMm || eye.diameterMm > m_config.maxPlausibleMm) {
      return false;
    }

    float ax, ay, az;
    pupillometry_detail::AxisFromAngles(azimuthDeg, elevationDeg, ax, ay, az);

    const float cosine = pupillometry_detail::CosineToAxis(eye, ax, ay, az);
    if (cosine < m_config.minCorrectionCosine) {
      return false;
    }

    outMm = eye.diameterMm / cosine;
    return std::isfinite(outMm);
  }

  PupillometryResult Process(const PupilFrame &frame, int64_t nowUs) {
    PupillometryResult out;
    out.timestampUs = frame.timestampUs;

    if (NearBlink(frame.timestampUs)) {
      return out;
    }

    float leftMm = 0.0f, rightMm = 0.0f;
    const bool haveLeft = CorrectEye(frame.left, m_config.leftCameraAzimuthDeg, m_config.leftCameraElevationDeg, leftMm);
    const bool haveRight = CorrectEye(frame.right, m_config.rightCameraAzimuthDeg, m_config.rightCameraElevationDeg, rightMm);

    if (!haveLeft && !haveRight) {
      return out;
    }

    // Averaging both eyes cuts high-frequency noise by roughly a third on real data, and the two are
    // physiologically yoked so there is no signal lost by doing it.
    out.contributingEyes = (haveLeft ? 1 : 0) + (haveRight ? 1 : 0);
    out.correctedMm = (haveLeft && haveRight) ? (leftMm + rightMm) * 0.5f : (haveLeft ? leftMm : rightMm);
    out.valid = true;

    // low-pass (DON'T TOUCH the baseline or activity index with this, they are already slow enough)
    if (!m_haveSmoothed) {
      m_smoothed = out.correctedMm;
      m_haveSmoothed = true;
      m_smoothedAtUs = frame.timestampUs;
    } else {
      const float dt = static_cast<float>(frame.timestampUs - m_smoothedAtUs) / 1e6f;
      if (dt > 0.0f) {
        const float tau = 1.0f / (6.2831853f * (m_config.lowPassHz > 0.0f ? m_config.lowPassHz : 1.0f));
        const float alpha = dt / (dt + tau);
        m_smoothed += (out.correctedMm - m_smoothed) * alpha;
        m_smoothedAtUs = frame.timestampUs;
      }
    }
    out.smoothedMm = m_smoothed;

    //rolling baseline
    m_baseline.push_back({frame.timestampUs, m_smoothed});
    const int64_t baselineSpanUs = static_cast<int64_t>(m_config.baselineWindowSeconds * 1e6f);
    while (!m_baseline.empty() && (frame.timestampUs - m_baseline.front().timestampUs) > baselineSpanUs) {
      m_baseline.pop_front();
    }

    // A median rather than a mean: it ignores the slow light-reflex excursions that would otherwise
    // drag the reference around.
    if ((frame.timestampUs - m_baseline.front().timestampUs) >= baselineSpanUs / 2) {
      std::vector<float> values;
      values.reserve(m_baseline.size());
      for (const Entry &e : m_baseline) {
        values.push_back(e.value);
      }
      out.baselineMm = pupillometry_detail::Median(std::move(values));
      out.baselineReady = true;
      out.deltaMm = m_smoothed - out.baselineMm;
      out.relative = out.baselineMm > 1e-3f ? out.deltaMm / out.baselineMm : 0.0f;
    }

    // activity index 
    m_activity.push_back({frame.timestampUs, out.correctedMm});
    const int64_t activitySpanUs = static_cast<int64_t>(m_config.activityWindowSeconds * 1e6f);
    while (!m_activity.empty() && (frame.timestampUs - m_activity.front().timestampUs) > activitySpanUs) {
      m_activity.pop_front();
    }

    if (m_activity.size() >= 32 && (frame.timestampUs - m_activity.front().timestampUs) >= activitySpanUs / 2) {
      out.activityIndex = ComputeActivityIndex();
      out.activityReady = true;
    }

    return out;
  }

  // Counts abrupt dilations per second.
  // This follows the shape of Duchowski et al.'s Index of Pupillary Activity: take a wavelet transform
  // of the diameter signal, threshold the detail coefficients against a noise estimate, and count what
  // survives. A Haar basis stands in for the paper's Symlet-16 -- coarser, but it keeps the property
  // that matters here, which is responding to the rate of sudden changes rather than the absolute
  // level, and therefore riding out slow luminance ramps that would swamp raw diameter.
  float ComputeActivityIndex() const {
    if (m_activity.size() < 16) {
      return 0.0f;
    }

    const size_t n = m_activity.size() - 1;
    std::vector<float> detail;
    detail.reserve(n);
    for (size_t i = 0; i < n; ++i) {
      detail.push_back((m_activity[i].value - m_activity[i + 1].value) * 0.70710678f);
    }

    // Robust noise estimate: MAD scaled to a standard deviation.
    std::vector<float> magnitudes;
    magnitudes.reserve(detail.size());
    for (float d : detail) {
      magnitudes.push_back(std::fabs(d));
    }
    const float sigma = pupillometry_detail::Median(std::move(magnitudes)) / 0.6745f;
    if (!(sigma > 0.0f)) {
      return 0.0f;
    }

    // Universal threshold.
    const float threshold = sigma * std::sqrt(2.0f * std::log(static_cast<float>(detail.size())));

    int count = 0;
    for (float d : detail) {
      if (std::fabs(d) > threshold) {
        ++count;
      }
    }

    const float spanSeconds = static_cast<float>(m_activity.back().timestampUs - m_activity.front().timestampUs) / 1e6f;
    return spanSeconds > 0.0f ? static_cast<float>(count) / spanSeconds : 0.0f;
  }

  struct Entry {
    int64_t timestampUs;
    float value;
  };

  PupillometryConfig m_config;
  std::deque<PupilFrame> m_pending;
  std::deque<Entry> m_baseline;
  std::deque<Entry> m_activity;

  float m_smoothed = 0.0f;
  int64_t m_smoothedAtUs = 0;
  bool m_haveSmoothed = false;
  int64_t m_lastEmitUs = 0;
  int64_t m_lastBlinkUs = 0;
};

// Fits the camera axis for one eye.
//
// The foreshortening correction is only as good as the axis it is told about, and that axis is a
// property of how the camera sits in a particular headset. Rather than hardcoding a guess, accumulate
// samples while the wearer looks around and solve for the axis that leaves no residual relationship
// between gaze angle and apparent diameter -- if the correction is right, the two become independent.
class PupilCameraAxisCalibrator {
public:
  // Roughly two minutes at 120 Hz. Far more than the fit needs, and it stops a forgotten calibration
  // from growing without bound.
  static constexpr size_t k_maxSamples = 16000;

  void Add(const PupilEyeSample &eye, float minPlausibleMm = 1.0f, float maxPlausibleMm = 9.0f) {
    if (m_samples.size() >= k_maxSamples) {
      return;
    }
    if (!eye.diameterValid || eye.blinking || !eye.gazeValid) {
      return;
    }
    if (!std::isfinite(eye.diameterMm) || eye.diameterMm < minPlausibleMm || eye.diameterMm > maxPlausibleMm) {
      return;
    }
    m_samples.push_back(eye);
  }

  size_t Count() const { return m_samples.size(); }
  void Reset() { m_samples.clear(); }

  // Grid search over plausible camera placements. Returns false if there is not enough data, or if the
  // wearer never looked around enough for the fit to mean anything.
  //
  // outResidual is the coefficient of variation of the corrected diameter at the chosen axis: lower is
  // better, and it will not reach zero on real data because pupil size genuinely varies. Compare it
  // against the uncorrected CV to judge whether the fit bought anything.
  bool Solve(float &outAzimuthDeg, float &outElevationDeg, float &outResidual, float &outGazeSpreadDeg) const {
    if (m_samples.size() < 200) {
      return false;
    }

    // outGazeSpreadDeg is the smaller of the yaw and pitch spreads, so this rejects both "never looked
    // around" and "only looked along one axis".
    outGazeSpreadDeg = GazeSpreadDegrees();
    if (outGazeSpreadDeg < 8.0f) {
      return false;
    }

    bool found = false;
    float best = 0.0f;

    for (int az = -40; az <= 40; az += 2) {
      for (int el = -40; el <= 40; el += 2) {
        float residual = 0.0f;
        if (!ResidualFor(static_cast<float>(az), static_cast<float>(el), residual)) {
          continue;
        }
        if (!found || residual < best) {
          found = true;
          best = residual;
          outAzimuthDeg = static_cast<float>(az);
          outElevationDeg = static_cast<float>(el);
        }
      }
    }

    outResidual = best;
    return found;
  }

private:
  // Returns the SMALLER of the yaw and pitch spreads, in degrees.
  float GazeSpreadDegrees() const {
    constexpr float k_radToDeg = 180.0f / 3.14159265358979323846f;
    float minYaw = 1e9f, maxYaw = -1e9f, minPitch = 1e9f, maxPitch = -1e9f;

    for (const PupilEyeSample &s : m_samples) {
      const float n = std::sqrt(s.gazeDirX * s.gazeDirX + s.gazeDirY * s.gazeDirY + s.gazeDirZ * s.gazeDirZ);
      if (n < 1e-6f) {
        continue;
      }
      const float yaw = std::atan2(s.gazeDirX / n, s.gazeDirZ / n) * k_radToDeg;
      // Clamped with ternaries rather than std::min/max: this header gets included in translation units
      // that pull in windows.h without NOMINMAX, where those names are macros and will not compile.
      const float sinPitch = s.gazeDirY / n;
      const float sinPitchClamped = sinPitch > 1.0f ? 1.0f : (sinPitch < -1.0f ? -1.0f : sinPitch);
      const float pitch = std::asin(sinPitchClamped) * k_radToDeg;

      minYaw = yaw < minYaw ? yaw : minYaw;
      maxYaw = yaw > maxYaw ? yaw : maxYaw;
      minPitch = pitch < minPitch ? pitch : minPitch;
      maxPitch = pitch > maxPitch ? pitch : maxPitch;
    }

    const float yawSpread = maxYaw - minYaw;
    const float pitchSpread = maxPitch - minPitch;
    return yawSpread < pitchSpread ? yawSpread : pitchSpread;
  }

  // Scores a candidate axis by how CONSTANT it makes the corrected diameter, as a coefficient of variation.
  bool ResidualFor(float azimuthDeg, float elevationDeg, float &outResidual) const {
    float ax, ay, az;
    pupillometry_detail::AxisFromAngles(azimuthDeg, elevationDeg, ax, ay, az);

    double sum = 0.0, sumSq = 0.0;
    size_t n = 0;

    for (const PupilEyeSample &s : m_samples) {
      const float c = pupillometry_detail::CosineToAxis(s, ax, ay, az);
      if (c < 0.5f) {
        return false; // this axis puts samples implausibly far off-axis
      }
      const double corrected = static_cast<double>(s.diameterMm) / c;
      sum += corrected;
      sumSq += corrected * corrected;
      ++n;
    }

    if (n < 2) {
      return false;
    }

    const double mean = sum / static_cast<double>(n);
    if (mean <= 1e-6) {
      return false;
    }

    const double rawVariance = sumSq / static_cast<double>(n) - mean * mean;
    const double variance = rawVariance > 0.0 ? rawVariance : 0.0;
    outResidual = static_cast<float>(std::sqrt(variance) / mean);
    return true;
  }

  std::vector<PupilEyeSample> m_samples;
};
