#pragma once

#include "aston_context.h"

struct AstonManager_t {
  void *vfptr;
  AstonContext_t *contexts[2]; // 0 = Right, 1 = Left
  char unk0[8];
};
static_assert(sizeof(AstonManager_t) == 0x20, "Size of AstonManager_t is not 0x20 bytes!");

AstonManager_t *(*AstonManager__getSingleton)(); // TODO
