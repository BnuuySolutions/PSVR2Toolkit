#pragma once

#include "psvr2tk_capi_private.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

#define PSVR2TK_INLINE_PRIVATE_HELPER(ret, name, args_decl, args_pass)                                                                                         \
  inline decltype(&name) p_##name;                                                                                                                             \
  inline ret name args_decl { return p_##name args_pass; }

PSVR2TK_CAPI_PRIVATE_FUNCTIONS(PSVR2TK_INLINE_PRIVATE_HELPER)

#undef PSVR2TK_INLINE_PRIVATE_HELPER

// Returns how many private entry points could not be resolved. See the note on the public loader in
// psvr2tk_capi_loader.h -- an unresolved symbol here is a null pointer waiting to be called.
inline int psvr2_toolkit_private_loader_init_functions(void *handle) {
  int missing = 0;

#ifdef _WIN32
#define PSVR2TK_LOAD_PRIVATE_FUNC(ret, name, args_decl, args_pass)                                                                                             \
  p_##name = handle ? reinterpret_cast<decltype(p_##name)>(GetProcAddress(static_cast<HMODULE>(handle), #name)) : nullptr;                                     \
  if (!p_##name)                                                                                                                                               \
    ++missing;
#else
#define PSVR2TK_LOAD_PRIVATE_FUNC(ret, name, args_decl, args_pass)                                                                                             \
  p_##name = handle ? reinterpret_cast<decltype(p_##name)>(dlsym(handle, #name)) : nullptr;                                                                    \
  if (!p_##name)                                                                                                                                               \
    ++missing;
#endif

  PSVR2TK_CAPI_PRIVATE_FUNCTIONS(PSVR2TK_LOAD_PRIVATE_FUNC)

#undef PSVR2TK_LOAD_PRIVATE_FUNC

  return missing;
}
