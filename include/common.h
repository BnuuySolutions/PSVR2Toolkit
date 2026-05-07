#pragma once

#include <cstdint>

#include "hmd2_gaze.h"
#include "pad_trigger_effect.h"

constexpr int k_senseChunkSize = 32;
constexpr int k_senseSampleRate = 3000;

enum class VRControllerType : uint8_t {
  Left = 0,
  Right = 1,
  Both = 2
};

#pragma pack(push, 1)
struct TriggerEffectCommandPayload {
  VRControllerType controllerType;
  ScePadTriggerEffectCommand command;
};

// TODO: the calibration stuff needs a better home and likely naming convention change

enum GazeCalibrationReportMode : uint16_t {
  None = 0,
  StartCalibration = 1,
  CollectCalibrationPoint = 2,
  DiscardCalibrationPoint = 3,
  ComputeAndApplyCalibration = 4,
  RetrieveCalibrationData = 5,
  StopCalibration = 6,
  SetEnabledEye = 7,
  HardwareCalibrationRetrieve = 8
};

enum GazeCalibrationStatus : uint16_t {
  EyetrackingInactive = 0,
  EyetrackingActive = 1,
  CalibrationReady = 2,
  DSPBusy = 3,
  Computing = 4,
  ComputeSucceeded = 5,
  ComputeFailed = 6,
  SettingEye = 7
};

enum GazeCalibrationResult : uint8_t {
  Success = 0,
  Failure = 1,
  Discarded = 2,
  Waiting = 3
};

struct GazeCalibrationPacket {
  float x;
  float y;
  float z;
  union {
    GazeCalibrationResult result;
    hmd2_gaze_enabled_eye_t eyeEnabled;
  };
};

struct GazeCalibrationCommand {
  union {
    GazeCalibrationReportMode reportMode;
    GazeCalibrationStatus status;
  };
  GazeCalibrationPacket payload;
};

struct HeadsetRumbleCommand {
  uint8_t rumbleHz;
};

enum class DriverCommandType : uint32_t {
  GazeCalibrationSet = 0,
  GazeCalibrationGet = 1,
  HeadsetRumbleSet = 2
};

struct DriverCommand {
  DriverCommandType type;
  volatile bool isFulfilled;
  union {
    GazeCalibrationCommand gazeCalibration;
    HeadsetRumbleCommand headsetRumble;
  };
};
#pragma pack(pop)