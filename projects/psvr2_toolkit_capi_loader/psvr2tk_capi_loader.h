#pragma once

#include "psvr2tk_capi.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

#define PSVR2TK_INLINE_HELPER(ret, name, args_decl, args_pass) \
  inline decltype(&name) p_##name; \
  inline ret name args_decl { return p_##name args_pass; }

PSVR2TK_CAPI_FUNCTIONS(PSVR2TK_INLINE_HELPER)

#undef PSVR2TK_INLINE_HELPER

inline void psvr2_toolkit_loader_init_functions(void* handle) {
#ifdef _WIN32
#define PSVR2TK_LOAD_FUNC(ret, name, args_decl, args_pass) p_##name = reinterpret_cast<decltype(p_##name)>(GetProcAddress(static_cast<HMODULE>(handle), #name));
#else
#define PSVR2TK_LOAD_FUNC(ret, name, args_decl, args_pass) p_##name = reinterpret_cast<decltype(p_##name)>(dlsym(handle, #name));
#endif

    PSVR2TK_CAPI_FUNCTIONS(PSVR2TK_LOAD_FUNC)

#undef PSVR2TK_LOAD_FUNC
}

#ifdef __cplusplus
extern "C" {
#endif
  PSVR2TK_EXPORT void* psvr2_toolkit_loader_get_module_handle();
  PSVR2TK_EXPORT size_t psvr2_toolkit_loader_get_module_path(char* buffer, size_t bufferSize);
#ifdef __cplusplus
}
#endif