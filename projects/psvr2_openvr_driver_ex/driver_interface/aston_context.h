#pragma once

struct AstonContext_t {
  void *vfptr;
  char unk0[0x28];
  int handle; // libpad handle
  char unk1[0xEFE8C];
};
static_assert(sizeof(AstonContext_t) == 0xEFEC0, "Size of AstonManager_t is not 0xEFEC0 bytes!");
