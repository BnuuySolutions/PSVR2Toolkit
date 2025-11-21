#include "hmd_driver_loader.h"

#include <shlwapi.h>

#define HMD_DLL_NAME L"driver_playstation_vr2_orig.dll"

extern "C" IMAGE_DOS_HEADER __ImageBase;

namespace psvr2_toolkit {

  // Allows the C++ standard library load the original HMD driver automatically for us.
  class HmdDriverLoaderInitializer {
  public:
    HmdDriverLoaderInitializer() {
      HmdDriverLoader::Instance();
    }
  };
  HmdDriverLoaderInitializer __initializer;

  HmdDriverLoader *HmdDriverLoader::m_pInstance = nullptr;

  HmdDriverLoader::HmdDriverLoader()
    : pfnHmdDriverFactory(nullptr)
    , m_hModule(nullptr)
  {
    wchar_t pszHmdDllPath[MAX_PATH] = { 0 };
    if (GetHmdDllPath(pszHmdDllPath)) {
      m_hModule = LoadLibraryW(pszHmdDllPath);
      if (m_hModule) {
        pfnHmdDriverFactory = decltype(pfnHmdDriverFactory)(GetProcAddress(m_hModule, "HmdDriverFactory"));
      }
    }
  }

  HmdDriverLoader *HmdDriverLoader::Instance() {
    if (!m_pInstance) {
      m_pInstance = new HmdDriverLoader;
    }

    return m_pInstance;
  }

  uintptr_t HmdDriverLoader::GetBaseAddress() {
    return reinterpret_cast<uintptr_t>(m_hModule);
  }

  bool HmdDriverLoader::GetHmdDllPath(wchar_t *pszHmdDllPath) {
    if (!pszHmdDllPath) {
      return false;
    }

    wchar_t pszPath[MAX_PATH] = {0};
    DWORD dwLength = GetModuleFileNameW(reinterpret_cast<HINSTANCE>(&__ImageBase), pszPath, MAX_PATH);
    if (dwLength > 0 && dwLength < MAX_PATH) {
      if (PathRemoveFileSpecW(pszPath)) {
        if (PathCombineW(pszHmdDllPath, pszPath, HMD_DLL_NAME)) {
          return true;
        }
      }
    }

    return false;
  }

} // psvr2_toolkit
