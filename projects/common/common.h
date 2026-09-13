#pragma once

#include <atomic>
#include <cstdint>
#include <type_traits>

#include "hmd2_gaze.h"
#include "pad_trigger_effect.h"

#ifndef PSVR2TK_EXPORT
#ifdef _WIN32
#define PSVR2TK_EXPORT __declspec(dllexport)
#else
#define PSVR2TK_EXPORT __attribute__((visibility("default")))
#endif
#endif

constexpr int k_senseChunkSize = 32;
constexpr int k_senseSampleRate = 3000;

enum class VRControllerType : uint8_t { Left = 0, Right = 1, Both = 2 };

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

enum GazeCalibrationResult : uint8_t { Success = 0, Failure = 1, Discarded = 2, Waiting = 3 };

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

// Processed pupillometry.
//
// Plain data with fixed-width fields so non-C++ consumers can marshal it: the Baballonia module and the
// Unity calibration app both P/Invoke this API from managed code.
//
// Prefer activityIndex over deltaMm. There is no display-luminance signal anywhere in the headset
// telemetry, so absolute diameter cannot be separated from the scene simply getting brighter and activityIndex counts abrupt dilations instead and rides out slow light changes. read pupillometry.h!!!!!!
struct Psvr2tkPupillometry {
  int64_t timestamp;

  uint32_t isValid;
  uint32_t isBaselineReady;
  uint32_t isActivityReady;
  uint32_t contributingEyes; // 0, 1 or 2

  float correctedMm; // foreshortening-corrected, binocular where both eyes were usable
  float smoothedMm;
  float baselineMm;
  float deltaMm;       // smoothedMm - baselineMm
  float relative;      // deltaMm / baselineMm
  float activityIndex; // abrupt dilations per second
};

struct HeadsetRumbleCommand {
  uint8_t rumbleHz;
};

struct UsbConnectionCommand {
  bool isConnected;
};

struct TriggerEffectCommand {
  int32_t slot;
  TriggerEffectCommandPayload payload;
};

enum class DriverCommandType : uint32_t {
  GazeCalibrationSet = 0,
  GazeCalibrationGet = 1,
  HeadsetRumbleSet = 2,
  UsbConnectionStateSet = 3,
  TriggerEffectSet = 4
};

struct DriverCommand {
  DriverCommandType type;

  // Cross-process completion handshake, written by the driver and polled by the submitting process
  // without holding the command mutex.
  //
  // Declared as a plain bool and accessed through std::atomic_ref at every site (see
  // custom_share_manager.cpp) rather than as std::atomic<bool>: std::atomic is not copyable, and this
  // struct is assigned by value into and out of the shared-memory ring. `volatile`, which this used to
  // be, is not a synchronisation primitive and carried no ordering guarantees at all.
  bool isFulfilled;

  union {
    GazeCalibrationCommand gazeCalibration;
    HeadsetRumbleCommand headsetRumble;
    UsbConnectionCommand usbConnection;
    TriggerEffectCommand triggerEffect;
  };
};
#pragma pack(pop)

static_assert(std::is_trivially_copyable_v<DriverCommand>, "DriverCommand is copied by value through shared memory and must stay trivially copyable!");
// std::atomic_ref is C++20. This header ships in the public CAPI dist include folder, where consumers
// may still be C++17, so the check is conditional -- the driver and libcustomshare build as C++23 and
// will evaluate it.
#if defined(__cpp_lib_atomic_ref)
static_assert(std::atomic_ref<bool>::is_always_lock_free, "std::atomic_ref<bool> must be lock-free to be usable across processes!");
#endif
