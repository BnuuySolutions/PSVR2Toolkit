#pragma once

#include <windows.h>
#include <tlhelp32.h>
#include <stdarg.h>
#include <stdio.h>
#include <openvr_driver.h>

namespace psvr2_toolkit {

  class Util {
  public:
    static bool StartsWith(const char *a, const char *b) {
      return strncmp(a, b, strlen(b)) == 0;
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

    static void DriverLogVarArgs(const char* pMsgFormat, va_list args)
    {
      char buf[1024] = {};
      vsnprintf_s(buf, sizeof(buf), pMsgFormat, args);

      vr::VRDriverLog()->Log(buf);
    }

    static void DriverLog(const char* pMsgFormat, ...)
    {
      va_list args;
      va_start(args, pMsgFormat);

      DriverLogVarArgs(pMsgFormat, args);

      va_end(args);
    }
  };

} // psvr2_toolkit
