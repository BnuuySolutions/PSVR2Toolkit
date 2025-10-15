#include "eyelid_estimator.h"

#include <algorithm>
#include <cmath>

namespace psvr2_toolkit {

  // parameters
  static constexpr float openLR = 0.001f; // slow open adaptation
  static constexpr float closedLR = 0.01f;  // faster closed adaptation
  static constexpr float smoothAlpha = 0.2f;   // EMA smoothing

  EyelidEstimator::EyelidEstimator()
    : m_openDia(4.0f)
    , m_closedDia(2.0f)
    , m_sensorYOpen(0.55f)
    , m_sensorYClosed(0.45f)
    , m_lastOpenness(1.0f)
  {}

  bool EyelidEstimator::IsNeutral(const Hmd2GazeEye &eye) const {
    if (eye.isGazeDirValid == HMD2_BOOL_FALSE) {
      return false;
    }

    // Forward assumed to be (0,0,1). Require within ~20deg of forward.
    return eye.gazeDirNorm.z > 0.94f;
  }

  float EyelidEstimator::Estimate(const Hmd2GazeEye &eye) {
    if (eye.isBlinkValid && eye.blink == HMD2_BOOL_TRUE) {
      // Update CLOSED reference on blink
      if (eye.isPupilDiaValid) {
        m_closedDia = m_closedDia * (1.0f - closedLR) +
          eye.pupilDiaMm * closedLR;
      }
      if (eye.isPupilPosInSensorValid) {
        m_sensorYClosed = m_sensorYClosed * (1.0f - closedLR) +
          eye.pupilPosInSensor.y * closedLR;
      }
    } else if (eye.isPupilDiaValid && eye.isPupilPosInSensorValid && IsNeutral(eye)) {
      // Update OPEN reference only if gaze is neutral
      bool stable = std::fabs(eye.pupilDiaMm - m_openDia) < 1.0f;
      if (stable) {
        m_openDia = m_openDia * (1.0f - openLR) +
          eye.pupilDiaMm * openLR;
        m_sensorYOpen = m_sensorYOpen * (1.0f - openLR) +
          eye.pupilPosInSensor.y * openLR;
      }
    }

    float opennessRaw = m_lastOpenness;
    if (eye.isBlinkValid && eye.blink == HMD2_BOOL_TRUE) {
      opennessRaw = 0.0f; // explicit blink
    } else if (eye.isPupilDiaValid && eye.isPupilPosInSensorValid) {
      float denomDia = std::max(1e-6f, m_openDia - m_closedDia);
      float normDia = (eye.pupilDiaMm - m_closedDia) / denomDia;
      normDia = std::clamp(normDia, 0.0f, 1.0f);

      float denomPos = std::max(1e-6f, m_sensorYOpen - m_sensorYClosed);
      float normPos = (eye.pupilPosInSensor.y - m_sensorYClosed) / denomPos;
      normPos = std::clamp(normPos, 0.0f, 1.0f);

      // Weight cues based on gaze vertical angle
      float gazeWeight = 1.0f;
      if (eye.isGazeDirValid) {
        float vertical = eye.gazeDirNorm.y; // +up, -down
        gazeWeight = std::clamp(1.0f - std::fabs(vertical) * 1.5f, 0.2f, 1.0f);
      }

      opennessRaw = gazeWeight * (0.65f * normDia + 0.35f * normPos)
        + (1.0f - gazeWeight) * normDia; // fallback to diameter when gaze extreme
    } else {
      // Missing data ? decay toward closed
      opennessRaw = m_lastOpenness * 0.9f;
    }

    float openness = m_lastOpenness * (1.0f - smoothAlpha) +
      opennessRaw * smoothAlpha;
    m_lastOpenness = openness;

    return openness; // 0 = closed, 1 = open
  }

}

