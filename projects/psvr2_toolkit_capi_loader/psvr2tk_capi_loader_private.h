#pragma once

#include "psvr2tk_capi_private.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

#define PSVR2TK_INLINE_PRIVATE_HELPER(ret, name, args_decl, args_pass) \
  inline decltype(&name) p_##name; \
  inline ret name args_decl { return p_##name args_pass; }

PSVR2TK_CAPI_PRIVATE_FUNCTIONS(PSVR2TK_INLINE_PRIVATE_HELPER)

#undef PSVR2TK_INLINE_PRIVATE_HELPER

inline void psvr2_toolkit_private_loader_init_functions(void* handle) {
#ifdef _WIN32
#define PSVR2TK_LOAD_PRIVATE_FUNC(ret, name, args_decl, args_pass) p_##name = reinterpret_cast<decltype(p_##name)>(GetProcAddress(static_cast<HMODULE>(handle), #name));
#else
#define PSVR2TK_LOAD_PRIVATE_FUNC(ret, name, args_decl, args_pass) p_##name = reinterpret_cast<decltype(p_##name)>(dlsym(handle, #name));
#endif

    PSVR2TK_CAPI_PRIVATE_FUNCTIONS(PSVR2TK_LOAD_PRIVATE_FUNC)

#undef PSVR2TK_LOAD_PRIVATE_FUNC
}
