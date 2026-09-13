#pragma once

#include "psvr2tk_capi.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

#define PSVR2TK_INLINE_HELPER(ret, name, args_decl, args_pass)                                                                                                 \
  inline decltype(&name) p_##name;                                                                                                                             \
  inline ret name args_decl { return p_##name args_pass; }

PSVR2TK_CAPI_FUNCTIONS(PSVR2TK_INLINE_HELPER)

#undef PSVR2TK_INLINE_HELPER

// Resolves every entry point and returns how many could NOT be resolved.
//
// Check the result. Every function above is an inline shim that jumps through its p_ pointer, so an
// unresolved symbol is not a benign missing feature -- it is a null pointer that the first caller
// executes, crashing with no module name and no message. The usual cause is a version mismatch: an
// application built against a newer header talking to an older psvr2_toolkit_capi.dll left behind by a
// partial install.
inline int psvr2_toolkit_loader_init_functions(void *handle) {
  int missing = 0;

#ifdef _WIN32
#define PSVR2TK_LOAD_FUNC(ret, name, args_decl, args_pass)                                                                                                     \
  p_##name = handle ? reinterpret_cast<decltype(p_##name)>(GetProcAddress(static_cast<HMODULE>(handle), #name)) : nullptr;                                     \
  if (!p_##name)                                                                                                                                               \
    ++missing;
#else
#define PSVR2TK_LOAD_FUNC(ret, name, args_decl, args_pass)                                                                                                     \
  p_##name = handle ? reinterpret_cast<decltype(p_##name)>(dlsym(handle, #name)) : nullptr;                                                                    \
  if (!p_##name)                                                                                                                                               \
    ++missing;
#endif

  PSVR2TK_CAPI_FUNCTIONS(PSVR2TK_LOAD_FUNC)

#undef PSVR2TK_LOAD_FUNC

  return missing;
}

#ifdef __cplusplus
extern "C" {
#endif
PSVR2TK_EXPORT void *psvr2_toolkit_loader_get_module_handle();
PSVR2TK_EXPORT size_t psvr2_toolkit_loader_get_module_path(char *buffer, size_t bufferSize);
#ifdef __cplusplus
}
#endif