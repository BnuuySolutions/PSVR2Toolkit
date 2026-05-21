#pragma once

#include <cstdint>

class AstonContext {
public:
  void **vftable;
  uint8_t unk0[0x28];
  int handle; // libpad handle
  uint8_t unk1[0xEFE8C];
};
static_assert(sizeof(AstonContext) == 0xEFEC0, "Size of AstonContext is not 0xEFEC0 bytes!");
