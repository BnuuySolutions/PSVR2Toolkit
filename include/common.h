#pragma once

#include <cstdint>

#include "pad_trigger_effect.h"

constexpr int MAX_SLOTS = 4;
constexpr int k_unSenseChunkSize = 32;
constexpr int k_unSenseSampleRate = 3000;

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
#pragma pack(pop)