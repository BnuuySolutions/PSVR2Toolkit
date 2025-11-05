#pragma once

#include <windows.h>

#include <cstdint>

namespace psvr2_toolkit {

  // Provides a thin interface between our proxy library and the original HMD driver.
  class HmdDriverLoader {
  public:
    HmdDriverLoader();

    static HmdDriverLoader *Instance();

    uintptr_t GetBaseAddress();

    void *(*pfnHmdDriverFactory)(const char *pInterfaceName, int *pReturnCode);

  private:
    static HmdDriverLoader *m_pInstance;

    HMODULE m_hModule;

    bool GetHmdDllPath(wchar_t *pszHmdDllPath);
  };

} // psvr2_toolkit
