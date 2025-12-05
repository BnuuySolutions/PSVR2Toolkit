#pragma once

#include <cstdint>

namespace psvr2_toolkit {
  enum class DeviceType
  {
    None,
    HMD,
    SenseControllerLeft,
    SenseControllerRight
  };

#pragma pack(push, 1)

  struct LDLed
  {
    uint8_t unk[3];
    uint16_t unk3;
    uint16_t leftmost;
    uint16_t rightmost;
    uint16_t topmost;
    uint16_t bottommost;
    uint32_t depth_or_size_unk;
    uint32_t depth_or_size_unk2;
    uint32_t depth_or_size_unk3;
    uint32_t depth_or_size_unk4;
    uint32_t confidence_unk;
    uint8_t unk2;
    uint16_t depth_or_size_unk5;
  };
  static_assert(sizeof(struct LDLed) == 0x24, "bad size");

  struct LDPayloadHeader
  {
    uint16_t magic;
    uint8_t unk1[6];
    uint32_t timestamp;
    uint32_t unk2;
    uint32_t unk3;
    uint32_t frame_counter;
    uint8_t unk[40];
  };
  static_assert(sizeof(struct LDPayloadHeader) == 0x40, "bad size");

  struct LDCamera
  {
    uint8_t num_leds;
    uint16_t unk;
    struct LDLed leds[256];
    uint8_t padding;
  };

  struct LDPayload
  {
    struct LDPayloadHeader header;
    struct LDCamera cameras[4];
  };
  static_assert(sizeof(struct LDPayload) == 0x9050, "bad size");

#pragma pack(pop)
}