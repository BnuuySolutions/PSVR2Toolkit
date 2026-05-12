#include "psvr2tk_capi_loader.h"
#include "../projects/libcustomshare/util.h"
#include <cstddef>

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

#include <fstream>
#include <string>
#include <cstring>
#include <filesystem>

extern "C" {

static void* g_psvr2tk_module_handle = nullptr;
static std::string g_psvr2tk_module_path;
static bool g_psvr2tk_loader_initialized = false;

static void psvr2_toolkit_loader_initialize() {
  if (g_psvr2tk_loader_initialized) return;
  g_psvr2tk_loader_initialized = true;

  try {
    std::filesystem::path temp_folder = GetSystemTempFolder();
    std::filesystem::path path_file = temp_folder / "psvr2tk_capi_path.txt";
    
    std::ifstream inFile(path_file);
    if (inFile.is_open()) {
      
      std::string dir_path;
      std::getline(inFile, dir_path);
      
      if (!dir_path.empty()) {
        std::filesystem::path capiPath(dir_path);
#ifdef _WIN32
        capiPath /= "psvr2_toolkit_capi.dll";
        if (IsRunningInWine()) {
          capiPath = WineGetDosFileName(capiPath.string());
        }
        
        g_psvr2tk_module_path = capiPath.string();
        g_psvr2tk_module_handle = LoadLibraryExA(g_psvr2tk_module_path.c_str(), NULL, LOAD_WITH_ALTERED_SEARCH_PATH);
#else
        capiPath /= "libpsvr2_toolkit_capi.so";
        g_psvr2tk_module_path = capiPath.string();
        g_psvr2tk_module_handle = dlopen(g_psvr2tk_module_path.c_str(), RTLD_NOW);
#endif
      }
    }
  } catch (...) {
  }
}

PSVR2TK_EXPORT void* psvr2_toolkit_loader_get_module_handle() {
  psvr2_toolkit_loader_initialize();
  return g_psvr2tk_module_handle;
}

PSVR2TK_EXPORT size_t psvr2_toolkit_loader_get_module_path(char* buffer, size_t bufferSize) {
  psvr2_toolkit_loader_initialize();
  size_t sizeWithNull = g_psvr2tk_module_path.size() + 1;
  if (!buffer || bufferSize < sizeWithNull) return g_psvr2tk_module_path.size();
  
  memcpy(buffer, g_psvr2tk_module_path.c_str(), sizeWithNull);

  return strlen(buffer);
}

} // extern "C"