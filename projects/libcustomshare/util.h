#pragma once

#include <string>
#include <stdexcept>

#ifdef _WIN32
#include <windows.h>

inline bool IsRunningInWine() {
  static bool is_running_in_wine = false;
  static bool wine_check_done = false;

  if (!wine_check_done) {
    is_running_in_wine = (GetProcAddress(GetModuleHandleA("ntdll"),
                                         "wine_get_version") != nullptr);
    wine_check_done = true;
  }

  return is_running_in_wine;
}

inline std::string WineGetDosFileName(const std::string &filename) {
  static LPWSTR (*CDECL wine_get_dos_file_name_ptr)(LPCSTR) =
      (decltype(wine_get_dos_file_name_ptr))GetProcAddress(
          GetModuleHandleA("KERNEL32"), "wine_get_dos_file_name");

  if (!wine_get_dos_file_name_ptr) {
    throw std::runtime_error("wine_get_dos_file_name not found");
  }

  LPWSTR dos_name_w = wine_get_dos_file_name_ptr(filename.c_str());
  if (!dos_name_w) {
    return "";
  }

  std::wstring ws(dos_name_w);
  std::string dos_name;
  dos_name.reserve(ws.length());
  for (wchar_t c : ws) {
    dos_name.push_back(static_cast<char>(c));
  }
  HeapFree(GetProcessHeap(), 0, dos_name_w);

  return dos_name;
}

inline std::string WineGetUnixFileName(const std::string &filename) {
  static LPSTR (*CDECL wine_get_unix_file_name_ptr)(LPCWSTR) =
      (decltype(wine_get_unix_file_name_ptr))GetProcAddress(
          GetModuleHandleA("KERNEL32"), "wine_get_unix_file_name");

  if (!wine_get_unix_file_name_ptr) {
    throw std::runtime_error("wine_get_unix_file_name not found");
  }

  std::wstring filename_w(filename.begin(), filename.end());
  LPSTR unix_name_a = wine_get_unix_file_name_ptr(filename_w.c_str());
  if (!unix_name_a) {
    return "";
  }

  std::string unix_name(unix_name_a);
  HeapFree(GetProcessHeap(), 0, unix_name_a);

  return unix_name;
}

// Retrieves the optimal temp directory depending on the environment (Native
// Win32 vs Wine)
inline std::string GetSystemTempFolder() {
  if (IsRunningInWine()) {
    try {
      std::string wine_tmp = WineGetDosFileName("/tmp");
      if (!wine_tmp.empty()) {
        // GetTempPathA returns a path with a trailing slash.
        // We append one here to keep the return format consistent.
        if (wine_tmp.back() != '\\' && wine_tmp.back() != '/') {
          wine_tmp += '\\';
        }
        return wine_tmp;
      }
    }
    catch (const std::exception &e) {
      MessageBoxA(NULL, "Unable to find /tmp. Make sure Z: is mounted.",
                  "libcustomshare error", 0);

      throw e;
    }
  }

  char temp_path[MAX_PATH];
  DWORD length = GetTempPathA(MAX_PATH, temp_path);

  if (length > 0 && length <= MAX_PATH) {
    return std::string(temp_path);
  }

  throw std::runtime_error("Unable to find a temp folder");
}

#else
inline std::string GetSystemTempFolder() {
  const char* tmpdir = getenv("TMPDIR");
  if (tmpdir && tmpdir[0] != '\0') {
    return std::string(tmpdir) + "/";
  }
  return "/tmp/";
}

#endif