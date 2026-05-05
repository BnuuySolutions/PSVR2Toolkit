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
    // Attempt to load the HMD DLL with the default name.
    // If we can't, try to find an alternative fallback name.
    if (!LoadHmdDll(false)) {
      LoadHmdDll(true);
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

  bool HmdDriverLoader::LoadHmdDll(bool useAltName) {
    wchar_t pszHmdDllPath[MAX_PATH] = { 0 };
    if (GetHmdDllPath(pszHmdDllPath, useAltName)) {
      m_hModule = LoadLibraryW(pszHmdDllPath);
      if (m_hModule) {
        pfnHmdDriverFactory = decltype(pfnHmdDriverFactory)(GetProcAddress(m_hModule, "HmdDriverFactory"));
        return true;
      }
    }

    return false;
  }

  bool HmdDriverLoader::GetHmdDllPath(wchar_t *pszHmdDllPath, bool useAltName) {
    if (!pszHmdDllPath) {
      return false;
    }

    wchar_t pszPath[MAX_PATH] = {0};
    DWORD dwLength = GetModuleFileNameW(reinterpret_cast<HINSTANCE>(&__ImageBase), pszPath, MAX_PATH);
    if (dwLength > 0 && dwLength < MAX_PATH) {
      if (PathRemoveFileSpecW(pszPath)) {
        if (!useAltName) {
          // Normal load behavior using exact standard name
          if (PathCombineW(pszHmdDllPath, pszPath, HMD_DLL_NAME)) {
            return true;
          }
        } else {
          // Fallback search behavior
          wchar_t searchPattern[MAX_PATH] = {0};
          if (PathCombineW(searchPattern, pszPath, L"driver_playstation_vr2*.dll")) {
            WIN32_FIND_DATAW findData;
            HANDLE hFind = FindFirstFileW(searchPattern, &findData);
            if (hFind != INVALID_HANDLE_VALUE) {
              do {
                if (findData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
                  continue;
                }

                // Ignore the PSVR2TK driver first to avoid conflicts.
                if (StrCmpIW(findData.cFileName, L"driver_playstation_vr2.dll") == 0) {
                  continue;
                }

                // Look for a file starting with "driver_playstation_vr2" (22 characters to compare)
                if (StrCmpNIW(findData.cFileName, L"driver_playstation_vr2", 22) == 0) {
                  
                  if (PathCombineW(pszHmdDllPath, pszPath, findData.cFileName)) {
                    FindClose(hFind);
                    return true;
                  }
                }
              } while (FindNextFileW(hFind, &findData));
              FindClose(hFind);
            }
          }
        }
      }
    }

    return false;
  }

} // psvr2_toolkit
