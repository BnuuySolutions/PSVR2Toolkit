#pragma once

#include "config.h"

#include <windows.h>
#include <tlhelp32.h>
#include <stdarg.h>
#include <stdio.h>
#include <openvr_driver.h>

#include <format>
#include <iostream>

namespace psvr2_toolkit {

  class Util {
  public:
    static bool StartsWith(const char *a, const char *b) {
      return strncmp(a, b, strlen(b)) == 0;
    }

    static bool IsRunningOnWine() {
#if !MOCK_IS_RUNNING_ON_WINE
      HMODULE hModule = GetModuleHandleW(L"ntdll.dll");
      if (!hModule) {
        return false;
      }
      return GetProcAddress(hModule, "wine_get_version") != nullptr;
#else
      return true;
#endif
    }

    static bool IsProcessRunning(DWORD dwProcessId) {
      HANDLE hSnapshot = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
      if (hSnapshot == INVALID_HANDLE_VALUE) {
        return false;
      }

      PROCESSENTRY32W pe;
      pe.dwSize = sizeof(pe);

      if (Process32FirstW(hSnapshot, &pe)) {
        do {
          if (pe.th32ProcessID == dwProcessId) {
            CloseHandle(hSnapshot);
            return true;
          }
        } while (Process32NextW(hSnapshot, &pe));
      }

      CloseHandle(hSnapshot);
      return false;
    }

    template <typename... Args>
    static void DriverLog(const char *format, const Args&... args) {
      std::string message = std::vformat(std::string_view(format), std::make_format_args(args...));
      vr::VRDriverLog()->Log(message.c_str());
    }
  };

} // psvr2_toolkit
