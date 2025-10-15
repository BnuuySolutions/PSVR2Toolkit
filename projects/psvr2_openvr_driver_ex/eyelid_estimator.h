#pragma once

#include "hmd2_gaze.h"

namespace psvr2_toolkit {

  class EyelidEstimator {
  public:
    EyelidEstimator();

    float Estimate(const Hmd2GazeEye &eye);

  private:
    float m_openDia;
    float m_closedDia;
    float m_sensorYOpen;
    float m_sensorYClosed;
    float m_lastOpenness;

    bool IsNeutral(const Hmd2GazeEye &eye) const;
  };

}
