#pragma once

#include <cstdint>

constexpr int MAX_SLOTS = 4;
constexpr int k_unSenseChunkSize = 32;
constexpr int k_unSenseSampleRate = 3000;

enum class VRControllerType : uint8_t {
  Left = 0,
  Right = 1,
  Both = 2
};

enum class TriggerEffectMode : uint8_t {
  Off = 0,
  Feedback = 1,
  Weapon = 2,
  Vibration = 3,
  MultiplePositionFeedback = 4,
  SlopeFeedback = 5,
  MultiplePositionVibration = 6
};

#pragma pack(push, 1)
struct TriggerEffectCommandPayload {
VRControllerType controllerType;
TriggerEffectMode mode;
union {
  struct { uint8_t position; uint8_t strength; } feedbackParam;
  struct { uint8_t startPosition; uint8_t endPosition; uint8_t strength; } weaponParam;
  struct { uint8_t position; uint8_t amplitude; uint8_t frequency; } vibrationParam;
  struct { uint8_t strength[10]; } multiplePositionFeedbackParam;
  struct { uint8_t startPosition; uint8_t endPosition; uint8_t startStrength; uint8_t endStrength; } slopeFeedbackParam;
  struct { uint8_t frequency; uint8_t amplitude[10]; } multiplePositionVibrationParam;
} commandData;
};
#pragma pack(pop)