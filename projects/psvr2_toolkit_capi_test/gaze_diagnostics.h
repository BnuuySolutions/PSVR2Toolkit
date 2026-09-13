#pragma once

// Read-only derived measurements over the gaze stream, for evaluating the quality of the tracker and the user's fit. This is not a driver-facing API
// Nothing here feeds back into the driver. The point is to answer, with a headset on, some questions that I wanted to test it out:
//
//   what pos_guide magnitude actually corresponds to a badly fitted headset?
//   what IPD do the gaze origins measure, and does it agree with what SteamVR already reports?
//   what are this user's blink rate and duration?

#include "hmd2_gaze.h"
#include "pupillometry.h"

#include <cmath>
#include <cstdint>
#include <deque>

namespace gaze_capture {

// Rolling mean/min/max over a bounded window.
class RollingStat {
public:
  explicit RollingStat(size_t capacity = 600) : m_capacity(capacity) {}

  void Add(float value) {
    if (!std::isfinite(value)) {
      return;
    }
    m_samples.push_back(value);
    m_sum += value;
    while (m_samples.size() > m_capacity) {
      m_sum -= m_samples.front();
      m_samples.pop_front();
    }
  }

  bool Empty() const { return m_samples.empty(); }
  size_t Count() const { return m_samples.size(); }
  float Mean() const { return m_samples.empty() ? 0.0f : static_cast<float>(m_sum / static_cast<double>(m_samples.size())); }

  float Min() const {
    float v = 0.0f;
    bool first = true;
    for (float s : m_samples) {
      if (first || s < v) {
        v = s;
        first = false;
      }
    }
    return v;
  }

  float Max() const {
    float v = 0.0f;
    bool first = true;
    for (float s : m_samples) {
      if (first || s > v) {
        v = s;
        first = false;
      }
    }
    return v;
  }

private:
  size_t m_capacity;
  std::deque<float> m_samples;
  double m_sum = 0.0;
};

struct BlinkStats {
  uint32_t blinks = 0;
  float lastDurationMs = 0.0f;
  float meanDurationMs = 0.0f;
  float blinksPerMinute = 0.0f;
  // Fraction of the session with at least one eye closed. The basis of PERCLOS-style drowsiness work.
  float closedFraction = 0.0f;
};

// Compact, copyable view of everything the UI displays.
//
// The UI used to copy the whole GazeDiagnostics under the lock each frame, which was already dragging
// several rolling buffers along and would now pull the calibrator's sample store too. Snapshotting the
// scalars keeps the locked section short and the per-frame cost flat.
struct GazeDiagnosticsSnapshot {
  uint64_t samples = 0;

  bool hasIpd = false;
  float ipdMeanMm = 0, ipdMinMm = 0, ipdMaxMm = 0;
  size_t ipdCount = 0;

  bool hasFit = false;
  float fitLeftMean = 0, fitRightMean = 0, fitLeftMax = 0, fitRightMax = 0, fitWorstNow = 0;

  BlinkStats blinks;

  bool hasConvergence = false;
  float convMeanMm = 0, convMinMm = 0, convMaxMm = 0;

  PupillometryResult pupil;
  PupillometryConfig pupilConfig;
  uint64_t pupilAccepted = 0, pupilRejected = 0;

  bool calibrating = false, calSolved = false;
  size_t calSamples = 0;
  float calLeftAz = 0, calLeftEl = 0, calRightAz = 0, calRightEl = 0;
  float calResidual = 0, calSpread = 0;
};

class GazeDiagnostics {
public:
  // Called once per new gaze frame, from the recorder thread.
  void Update(const hmd2_gaze_status_t &s, int64_t hostRecvUs) {
    if (m_firstSampleUs == 0) {
      m_firstSampleUs = hostRecvUs;
    }
    m_lastSampleUs = hostRecvUs;
    ++m_sampleCount;

    UpdateIpd(s);
    UpdateFit(s);
    UpdateBlink(s, hostRecvUs);
    UpdateConvergence(s);
    UpdatePupillometry(s);
  }

  // --- 4.3: measured IPD -----------------------------------------------------------------------
  // The gaze origins are eyeball rotation centres, so their separation is the user's interpupillary
  // distance, measured rather than dialled in.
  bool HasIpd() const { return !m_ipdMm.Empty(); }
  float IpdMeanMm() const { return m_ipdMm.Mean(); }
  float IpdMinMm() const { return m_ipdMm.Min(); }
  float IpdMaxMm() const { return m_ipdMm.Max(); }
  size_t IpdSampleCount() const { return m_ipdMm.Count(); }

  // --- 4.2: headset fit ------------------------------------------------------------------------
  bool HasFit() const { return !m_fitLeft.Empty() || !m_fitRight.Empty(); }
  float FitLeftMean() const { return m_fitLeft.Mean(); }
  float FitRightMean() const { return m_fitRight.Mean(); }
  float FitLeftMax() const { return m_fitLeft.Max(); }
  float FitRightMax() const { return m_fitRight.Max(); }
  float FitWorstNow() const { return m_fitWorstNow; }

  // --- 5.2: blink dynamics ---------------------------------------------------------------------
  const BlinkStats &Blinks() const { return m_blinkStats; }

  // --- 4.4: convergence ------------------------------------------------------------------------
  bool HasConvergence() const { return !m_convergenceMm.Empty(); }
  float ConvergenceMeanMm() const { return m_convergenceMm.Mean(); }
  float ConvergenceMinMm() const { return m_convergenceMm.Min(); }
  float ConvergenceMaxMm() const { return m_convergenceMm.Max(); }

  uint64_t SampleCount() const { return m_sampleCount; }

  // --- 5.3: pupillometry -------------------------------------------------------------------------
  const PupillometryResult &Pupil() const { return m_pupilResult; }
  uint64_t PupilAccepted() const { return m_pupilAccepted; }
  uint64_t PupilRejected() const { return m_pupilRejected; }

  void SetPupilConfig(const PupillometryConfig &config) {
    m_pupilConfig = config;
    m_pupil.Configure(config);
    m_pupil.Reset();
  }
  const PupillometryConfig &PupilConfig() const { return m_pupilConfig; }

  // Camera-axis calibration. Collect while the wearer looks around, then solve.
  void StartCalibration() {
    m_calLeft.Reset();
    m_calRight.Reset();
    m_calibrating = true;
    m_calSolved = false;
  }
  void StopCalibration() { m_calibrating = false; }
  bool IsCalibrating() const { return m_calibrating; }
  size_t CalibrationSamples() const { return m_calLeft.Count(); }
  bool CalibrationSolved() const { return m_calSolved; }
  float CalLeftAz() const { return m_calLeftAz; }
  float CalLeftEl() const { return m_calLeftEl; }
  float CalRightAz() const { return m_calRightAz; }
  float CalRightEl() const { return m_calRightEl; }
  float CalResidual() const { return m_calResidual; }
  float CalSpreadDeg() const { return m_calSpread; }

  // Returns false when there is not enough 2D gaze spread to identify the axes.
  bool SolveCalibration() {
    float lr = 0.0f, ls = 0.0f, rr = 0.0f, rs = 0.0f;
    const bool okL = m_calLeft.Solve(m_calLeftAz, m_calLeftEl, lr, ls);
    const bool okR = m_calRight.Solve(m_calRightAz, m_calRightEl, rr, rs);

    if (!okL || !okR) {
      m_calSolved = false;
      m_calSpread = ls < rs ? ls : rs;
      return false;
    }

    m_calResidual = (lr + rr) * 0.5f;
    m_calSpread = ls < rs ? ls : rs;
    m_calSolved = true;
    m_calibrating = false;

    m_pupilConfig.leftCameraAzimuthDeg = m_calLeftAz;
    m_pupilConfig.leftCameraElevationDeg = m_calLeftEl;
    m_pupilConfig.rightCameraAzimuthDeg = m_calRightAz;
    m_pupilConfig.rightCameraElevationDeg = m_calRightEl;
    m_pupil.Configure(m_pupilConfig);
    m_pupil.Reset();
    return true;
  }

  GazeDiagnosticsSnapshot Snapshot() const {
    GazeDiagnosticsSnapshot s;
    s.samples = m_sampleCount;

    s.hasIpd = HasIpd();
    s.ipdMeanMm = IpdMeanMm();
    s.ipdMinMm = IpdMinMm();
    s.ipdMaxMm = IpdMaxMm();
    s.ipdCount = IpdSampleCount();

    s.hasFit = HasFit();
    s.fitLeftMean = FitLeftMean();
    s.fitRightMean = FitRightMean();
    s.fitLeftMax = FitLeftMax();
    s.fitRightMax = FitRightMax();
    s.fitWorstNow = FitWorstNow();

    s.blinks = m_blinkStats;

    s.hasConvergence = HasConvergence();
    s.convMeanMm = ConvergenceMeanMm();
    s.convMinMm = ConvergenceMinMm();
    s.convMaxMm = ConvergenceMaxMm();

    s.pupil = m_pupilResult;
    s.pupilConfig = m_pupilConfig;
    s.pupilAccepted = m_pupilAccepted;
    s.pupilRejected = m_pupilRejected;

    s.calibrating = m_calibrating;
    s.calSolved = m_calSolved;
    s.calSamples = m_calLeft.Count();
    s.calLeftAz = m_calLeftAz;
    s.calLeftEl = m_calLeftEl;
    s.calRightAz = m_calRightAz;
    s.calRightEl = m_calRightEl;
    s.calResidual = m_calResidual;
    s.calSpread = m_calSpread;
    return s;
  }

  void Reset() { *this = GazeDiagnostics(); }

private:
  void UpdateIpd(const hmd2_gaze_status_t &s) {
    if (s.wearable.left.is_gaze_origin_valid != HMD2_GAZE_BOOL_TRUE || s.wearable.right.is_gaze_origin_valid != HMD2_GAZE_BOOL_TRUE) {
      return;
    }

    const float dx = s.wearable.left.gaze_origin_mm.x - s.wearable.right.gaze_origin_mm.x;
    const float dy = s.wearable.left.gaze_origin_mm.y - s.wearable.right.gaze_origin_mm.y;
    const float dz = s.wearable.left.gaze_origin_mm.z - s.wearable.right.gaze_origin_mm.z;
    const float separationMm = std::sqrt(dx * dx + dy * dy + dz * dz);

    // Human IPD spans roughly 51-77 mm. Anything outside a generous window is a bad sample, not a face.
    if (separationMm < 40.0f || separationMm > 90.0f) {
      return;
    }

    m_ipdMm.Add(separationMm);
  }

  void UpdateFit(const hmd2_gaze_status_t &s) {
    float worst = 0.0f;

    if (s.wearable.left.is_pos_guide_valid == HMD2_GAZE_BOOL_TRUE) {
      const float m = std::sqrt(s.wearable.left.pos_guide.x * s.wearable.left.pos_guide.x + s.wearable.left.pos_guide.y * s.wearable.left.pos_guide.y);
      m_fitLeft.Add(m);
      worst = std::fmax(worst, m);
    }

    if (s.wearable.right.is_pos_guide_valid == HMD2_GAZE_BOOL_TRUE) {
      const float m = std::sqrt(s.wearable.right.pos_guide.x * s.wearable.right.pos_guide.x + s.wearable.right.pos_guide.y * s.wearable.right.pos_guide.y);
      m_fitRight.Add(m);
      worst = std::fmax(worst, m);
    }

    m_fitWorstNow = worst;
  }

  void UpdateBlink(const hmd2_gaze_status_t &s, int64_t hostRecvUs) {
    const bool leftClosed = s.wearable.left.is_blink_valid == HMD2_GAZE_BOOL_TRUE && s.wearable.left.blink == HMD2_GAZE_BOOL_TRUE;
    const bool rightClosed = s.wearable.right.is_blink_valid == HMD2_GAZE_BOOL_TRUE && s.wearable.right.blink == HMD2_GAZE_BOOL_TRUE;
    const bool closed = leftClosed || rightClosed;

    if (closed) {
      m_closedSamples++;
      if (!m_wasClosed) {
        m_blinkStartUs = hostRecvUs;
      }
    } else if (m_wasClosed) {
      const float durationMs = static_cast<float>(hostRecvUs - m_blinkStartUs) / 1000.0f;

      // Ignore implausibly long closures: those are the eyes simply being shut, not a blink.
      if (durationMs > 0.0f && durationMs < 1000.0f) {
        m_blinkStats.blinks++;
        m_blinkStats.lastDurationMs = durationMs;
        m_blinkDurationSumMs += durationMs;
        m_blinkStats.meanDurationMs = static_cast<float>(m_blinkDurationSumMs / m_blinkStats.blinks);
      }
    }

    m_wasClosed = closed;

    const double elapsedMinutes = static_cast<double>(m_lastSampleUs - m_firstSampleUs) / 60e6;
    if (elapsedMinutes > 0.01) {
      m_blinkStats.blinksPerMinute = static_cast<float>(m_blinkStats.blinks / elapsedMinutes);
    }
    if (m_sampleCount > 0) {
      m_blinkStats.closedFraction = static_cast<float>(static_cast<double>(m_closedSamples) / static_cast<double>(m_sampleCount));
    }
  }

  static PupilEyeSample ToPupilSample(const hmd2_gaze_wearable_eye_t &eye) {
    PupilEyeSample out;
    out.diameterMm = eye.pupil_dia_mm;
    out.diameterValid = eye.is_pupil_dia_valid == HMD2_GAZE_BOOL_TRUE;
    out.blinking = eye.is_blink_valid == HMD2_GAZE_BOOL_TRUE && eye.blink == HMD2_GAZE_BOOL_TRUE;
    // Headset-space direction, +Z forward -- exactly what pupillometry.h expects.
    out.gazeDirX = eye.gaze_dir_norm.x;
    out.gazeDirY = eye.gaze_dir_norm.y;
    out.gazeDirZ = eye.gaze_dir_norm.z;
    out.gazeValid = eye.is_gaze_dir_valid == HMD2_GAZE_BOOL_TRUE;
    return out;
  }

  void UpdatePupillometry(const hmd2_gaze_status_t &s) {
    PupilFrame frame;
    // The headset clock, not the host one: it is the timebase the samples were measured against.
    frame.timestampUs = s.wearable.timestamp;
    frame.left = ToPupilSample(s.wearable.left);
    frame.right = ToPupilSample(s.wearable.right);

    if (m_calibrating) {
      m_calLeft.Add(frame.left, m_pupilConfig.minPlausibleMm, m_pupilConfig.maxPlausibleMm);
      m_calRight.Add(frame.right, m_pupilConfig.minPlausibleMm, m_pupilConfig.maxPlausibleMm);
    }

    PupillometryResult result;
    if (!m_pupil.Update(frame, result)) {
      return;
    }

    if (result.valid) {
      m_pupilResult = result;
      ++m_pupilAccepted;
    } else {
      // Mostly blink guard bands. A high rejection rate means either a lot of blinking or a fit problem.
      ++m_pupilRejected;
    }
  }

  void UpdateConvergence(const hmd2_gaze_status_t &s) {
    if (s.wearable.is_convergence_distance_valid != HMD2_GAZE_BOOL_TRUE) {
      return;
    }
    const float mm = s.foveated.convergence_distance_mm;
    if (!std::isfinite(mm) || mm <= 0.0f) {
      return;
    }
    m_convergenceMm.Add(mm);
  }

  RollingStat m_ipdMm;
  RollingStat m_fitLeft;
  RollingStat m_fitRight;
  RollingStat m_convergenceMm;
  float m_fitWorstNow = 0.0f;

  BlinkStats m_blinkStats;
  bool m_wasClosed = false;
  int64_t m_blinkStartUs = 0;
  double m_blinkDurationSumMs = 0.0;
  uint64_t m_closedSamples = 0;

  int64_t m_firstSampleUs = 0;
  int64_t m_lastSampleUs = 0;
  uint64_t m_sampleCount = 0;

  PupillometryConfig m_pupilConfig;
  PupillometryProcessor m_pupil;
  PupillometryResult m_pupilResult;
  uint64_t m_pupilAccepted = 0;
  uint64_t m_pupilRejected = 0;

  PupilCameraAxisCalibrator m_calLeft;
  PupilCameraAxisCalibrator m_calRight;
  bool m_calibrating = false;
  bool m_calSolved = false;
  float m_calLeftAz = 0.0f, m_calLeftEl = 0.0f;
  float m_calRightAz = 0.0f, m_calRightEl = 0.0f;
  float m_calResidual = 0.0f;
  float m_calSpread = 0.0f;
};

} // namespace gaze_capture
