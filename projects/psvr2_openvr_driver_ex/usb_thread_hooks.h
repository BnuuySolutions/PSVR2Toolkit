#pragma once

#include <cstdint>

namespace psvr2_toolkit {

  class UsbThreadHooks {
  public:
    static void InstallHooks();
  };

  // Stream

  void gazeStreamGetStatus();
  void gazeStreamSetEnabled(bool value);

  // User calibration

  void gazeUserCalibrationGetStatus();
  void gazeUserCalibrationStart();
  void gazeUserCalibrationCollect(float x, float y, float z);
  void gazeUserCalibrationDiscard(float x, float y, float z);
  void gazeUserCalibrationComputeAndApply();
  void gazeUserCalibrationGetData();
  void gazeUserCalibrationStop();
  void gazeUserCalibrationSetEnabledEye(uint8_t value);

  // Wrapper around gazeUserCalibrationCollect.
  void gazeUserCalibrationSetPoint(float x, float y, float z);

} // psvr2_toolkit
