// Off-device tests for the pupillometry pipeline (common/pupillometry.h).
//
// Signal characteristics below come from a real 120 Hz capture (EYE_TRACKING_PLAN.md 5.3): ~3.0 mm
// mean diameter, 0.019 mm sample-to-sample noise, 0.27 mm blink artifacts, and a measurable
// gaze-angle dependence that a fitted camera axis removes almost entirely. The tests reproduce those
// conditions rather than inventing convenient ones.

#include "pupillometry.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace {

int g_failures = 0;

void Check(bool condition, const std::string &name) {
  std::printf("  %s  %s\n", condition ? "pass" : "FAIL", name.c_str());
  if (!condition) {
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

constexpr int64_t k_frameUs = 8333; // 120 Hz, matching the hardware

struct Noise {
  uint32_t state = 0x9E3779B9u;
  float Next(float amplitude) {
    state = state * 1664525u + 1013904223u;
    const float unit = static_cast<float>((state >> 8) & 0xFFFF) / 65535.0f;
    return (unit * 2.0f - 1.0f) * amplitude;
  }
};

PupilEyeSample MakeEye(float diameterMm, float yawDeg = 0.0f, bool blinking = false, bool valid = true) {
  PupilEyeSample e;
  e.diameterMm = diameterMm;
  e.diameterValid = valid;
  e.blinking = blinking;
  e.gazeValid = true;
  // +Z forward, matching what the headset reports.
  const float rad = yawDeg * (3.14159265358979323846f / 180.0f);
  e.gazeDirX = std::sin(rad);
  e.gazeDirY = 0.0f;
  e.gazeDirZ = std::cos(rad);
  return e;
}

PupilFrame MakeFrame(int64_t t, float diameterMm, float yawDeg = 0.0f, bool blinking = false, bool valid = true) {
  PupilFrame f;
  f.timestampUs = t;
  f.left = MakeEye(diameterMm, yawDeg, blinking, valid);
  f.right = MakeEye(diameterMm, yawDeg, blinking, valid);
  return f;
}

} // namespace

int main() {
  std::printf("Pupillometry\n");

  // --- basic plumbing --------------------------------------------------------------------------

  {
    PupillometryProcessor p;
    PupillometryResult r;
    // Nothing should come out until the blink guard band has been covered.
    const bool early = p.Update(MakeFrame(1000, 3.0f), r);
    Check(!early, "no result before the blink guard band elapses");
  }

  {
    PupillometryProcessor p;
    PupillometryResult r;
    int64_t t = 0;
    bool got = false;
    for (int i = 0; i < 60; ++i) {
      t += k_frameUs;
      got = p.Update(MakeFrame(t, 3.0f), r) || got;
    }
    Check(got, "results start once the guard band is covered");
    Check(r.valid && r.contributingEyes == 2, "both eyes contribute when both are usable");
    CheckNear(r.correctedMm, 3.0f, 0.001f, "on-axis diameter passes through uncorrected");
  }

  // --- rejection -------------------------------------------------------------------------------

  {
    PupillometryProcessor p;
    PupillometryResult r;
    int64_t t = 0;
    int valid = 0, total = 0;
    for (int i = 0; i < 200; ++i) {
      t += k_frameUs;
      if (p.Update(MakeFrame(t, 3.0f, 0.0f, false, false), r)) {
        ++total;
        if (r.valid) {
          ++valid;
        }
      }
    }
    Check(total > 0 && valid == 0, "samples flagged invalid never produce a measurement");
  }

  {
    PupillometryProcessor p;
    PupillometryResult r;
    int64_t t = 0;
    int emittedNearBlink = 0;

    // 0.27 mm artifact around a blink, matching what the real capture shows.
    for (int i = 0; i < 300; ++i) {
      t += k_frameUs;
      // Artifact spans the guard band either side of the blink: 100 ms is 12 frames at 120 Hz.
      const bool blinking = (i >= 150 && i < 162);
      const bool nearBlink = (i >= 138 && i < 174);
      const float d = nearBlink ? 3.27f : 3.0f;
      if (p.Update(MakeFrame(t, d, 0.0f, blinking), r) && r.valid) {
        if (std::fabs(r.correctedMm - 3.0f) > 0.05f) {
          ++emittedNearBlink;
        }
      }
    }
    Check(emittedNearBlink == 0, "blink artifacts are rejected, not emitted");
  }

  {
    PupillometryProcessor p;
    PupillometryResult r;
    int64_t t = 0;
    int implausible = 0;
    for (int i = 0; i < 100; ++i) {
      t += k_frameUs;
      if (p.Update(MakeFrame(t, 25.0f), r) && r.valid) {
        ++implausible;
      }
    }
    Check(implausible == 0, "implausible diameters are rejected");
  }

  // --- foreshortening correction ----------------------------------------------------------------

  {
    // A pupil viewed 30 degrees off the camera axis appears cos(30) smaller. Correction must undo it.
    PupillometryConfig cfg;
    PupillometryProcessor p;
    p.Configure(cfg);

    PupillometryResult r;
    int64_t t = 0;
    const float trueDiameter = 3.0f;
    const float apparent = trueDiameter * std::cos(30.0f * 3.14159265f / 180.0f);

    for (int i = 0; i < 60; ++i) {
      t += k_frameUs;
      p.Update(MakeFrame(t, apparent, 30.0f), r);
    }
    CheckNear(r.correctedMm, trueDiameter, 0.01f, "off-axis foreshortening is corrected");
  }

  {
    // Beyond the cosine floor the correction would amplify noise; the sample must be dropped instead.
    PupillometryProcessor p;
    PupillometryResult r;
    int64_t t = 0;
    int emitted = 0;
    for (int i = 0; i < 100; ++i) {
      t += k_frameUs;
      if (p.Update(MakeFrame(t, 3.0f, 75.0f), r) && r.valid) {
        ++emitted;
      }
    }
    Check(emitted == 0, "samples beyond the correction limit are dropped");
  }

  // --- binocular fusion --------------------------------------------------------------------------

  {
    PupillometryProcessor p;
    PupillometryResult r;
    int64_t t = 0;
    for (int i = 0; i < 60; ++i) {
      t += k_frameUs;
      PupilFrame f = MakeFrame(t, 3.0f);
      f.right.diameterValid = false;
      p.Update(f, r);
    }
    Check(r.valid && r.contributingEyes == 1, "falls back to one eye when the other is unusable");
  }

  {
    PupillometryProcessor p;
    PupillometryResult r;
    int64_t t = 0;
    for (int i = 0; i < 60; ++i) {
      t += k_frameUs;
      PupilFrame f = MakeFrame(t, 0.0f);
      f.left.diameterMm = 2.8f;
      f.right.diameterMm = 3.2f;
      p.Update(f, r);
    }
    CheckNear(r.correctedMm, 3.0f, 0.001f, "both eyes are averaged");
  }

  // --- baseline ------------------------------------------------------------------------------------

  {
    PupillometryConfig cfg;
    cfg.baselineWindowSeconds = 4.0f;
    PupillometryProcessor p;
    p.Configure(cfg);

    PupillometryResult r;
    int64_t t = 0;
    Noise noise;

    // Two seconds is half the window, which is when the baseline becomes available.
    for (int i = 0; i < 600; ++i) {
      t += k_frameUs;
      p.Update(MakeFrame(t, 3.0f + noise.Next(0.02f)), r);
    }
    Check(r.baselineReady, "baseline becomes available after half a window");
    CheckNear(r.baselineMm, 3.0f, 0.05f, "baseline tracks a steady diameter");
    CheckNear(r.deltaMm, 0.0f, 0.05f, "delta is near zero with no change");

    // Now dilate by 0.4 mm and check the delta follows.
    for (int i = 0; i < 200; ++i) {
      t += k_frameUs;
      p.Update(MakeFrame(t, 3.4f + noise.Next(0.02f)), r);
    }
    Check(r.deltaMm > 0.15f, "a real dilation shows up as a positive delta");
    Check(r.relative > 0.05f, "relative change is reported");
  }

  // --- activity index ------------------------------------------------------------------------------

  {
    // A slow luminance-style ramp must NOT look like activity: that is the whole point of the index.
    PupillometryConfig cfg;
    cfg.activityWindowSeconds = 3.0f;
    PupillometryProcessor p;
    p.Configure(cfg);

    PupillometryResult r;
    int64_t t = 0;
    Noise noise;
    float rampActivity = 0.0f;

    for (int i = 0; i < 900; ++i) {
      t += k_frameUs;
      const float ramp = 3.0f + 1.5f * (static_cast<float>(i) / 900.0f); // 1.5 mm over 7.5 s
      if (p.Update(MakeFrame(t, ramp + noise.Next(0.02f)), r) && r.activityReady) {
        rampActivity = r.activityIndex;
      }
    }

    // Abrupt steps, in contrast, must register.
    PupillometryProcessor p2;
    p2.Configure(cfg);
    PupillometryResult r2;
    int64_t t2 = 0;
    Noise noise2;
    float stepActivity = 0.0f;

    for (int i = 0; i < 900; ++i) {
      t2 += k_frameUs;
      const float step = 3.0f + ((i / 30) % 2 ? 0.25f : 0.0f); // a jump every 0.25 s
      if (p2.Update(MakeFrame(t2, step + noise2.Next(0.02f)), r2) && r2.activityReady) {
        stepActivity = r2.activityIndex;
      }
    }

    std::printf("        slow ramp activity = %.2f/s, abrupt steps = %.2f/s\n", rampActivity, stepActivity);
    Check(stepActivity > rampActivity * 2.0f, "abrupt dilations register far above a slow luminance ramp");
  }

  // --- camera axis calibration ----------------------------------------------------------------------

  {
    // Synthesise an eye whose camera sits 20 degrees to the side, then check the fit recovers it.
    PupilCameraAxisCalibrator cal;
    Noise noise;
    const float trueAz = -20.0f;
    float ax, ay, az;
    pupillometry_detail::AxisFromAngles(trueAz, 0.0f, ax, ay, az);

    // Sweep BOTH axes. Yaw-only data cannot constrain the camera elevation -- see GazeSpreadDegrees.
    for (int i = 0; i < 800; ++i) {
      const float yaw = -25.0f + 50.0f * (static_cast<float>(i % 200) / 199.0f);
      const float pitch = -15.0f + 30.0f * (static_cast<float>((i * 7) % 200) / 199.0f);

      PupilEyeSample e;
      e.diameterValid = true;
      e.gazeValid = true;
      const float yr = yaw * (3.14159265358979323846f / 180.0f);
      const float pr = pitch * (3.14159265358979323846f / 180.0f);
      e.gazeDirX = std::sin(yr) * std::cos(pr);
      e.gazeDirY = std::sin(pr);
      e.gazeDirZ = std::cos(yr) * std::cos(pr);

      const float cosine = pupillometry_detail::CosineToAxis(e, ax, ay, az);
      e.diameterMm = 3.0f * cosine + noise.Next(0.01f); // what the camera would actually see
      cal.Add(e);
    }

    float fitAz = 0.0f, fitEl = 0.0f, residual = 1.0f, spread = 0.0f;
    const bool solved = cal.Solve(fitAz, fitEl, residual, spread);
    Check(solved, "calibrator solves with enough spread");
    if (solved) {
      std::printf("        fitted az=%.0f (true %.0f), el=%.0f, residual=%.3f, spread=%.1f deg\n", fitAz, trueAz, fitEl, residual, spread);
      CheckNear(fitAz, trueAz, 4.0f, "recovers the camera azimuth");
      Check(residual < 0.15f, "fit leaves little gaze-angle dependence");
    }
  }

  {
    // Looking in one direction the whole time cannot constrain the fit, and must be refused.
    PupilCameraAxisCalibrator cal;
    for (int i = 0; i < 800; ++i) {
      cal.Add(MakeEye(3.0f, 0.0f));
    }
    float a = 0, e = 0, res = 0, spread = 0;
    Check(!cal.Solve(a, e, res, spread), "refuses to fit when the wearer never looked around");
  }

  {
    PupilCameraAxisCalibrator cal;
    for (int i = 0; i < 50; ++i) {
      cal.Add(MakeEye(3.0f, static_cast<float>(i % 30) - 15.0f));
    }
    float a = 0, e = 0, res = 0, spread = 0;
    Check(!cal.Solve(a, e, res, spread), "refuses to fit on too few samples");
  }

  {
    PupilCameraAxisCalibrator cal;
    cal.Add(MakeEye(3.0f, 0.0f, true));         // blinking
    cal.Add(MakeEye(3.0f, 0.0f, false, false)); // invalid
    cal.Add(MakeEye(25.0f, 0.0f));              // implausible
    Check(cal.Count() == 0, "calibrator ignores blinking, invalid and implausible samples");
  }

  std::printf("\n%s (%d failure%s)\n", g_failures == 0 ? "PASSED" : "FAILED", g_failures, g_failures == 1 ? "" : "s");
  return g_failures == 0 ? 0 : 1;
}
