#include <windows.h>

BOOL RealDllMain(void) {
  MessageBoxW(
    NULL,
    L"Hi there! You tried launching the official PlayStation VR2 App, which can cause issues while PlayStation VR2 Toolkit is installed."
    L"\n\n"
    L"Launching the official PlayStation VR2 App has been disabled, to prevent such issues from occuring.",
    L"[TEMP] PlayStation VR2 Toolkit",
    MB_ICONERROR | MB_OK);

  TerminateProcess(GetCurrentProcess(), 0);

  return FALSE;
}

FARPROC GetFileVersionInfoA_orig = 0;
FARPROC GetFileVersionInfoByHandle_orig = 0;
FARPROC GetFileVersionInfoExA_orig = 0;
FARPROC GetFileVersionInfoExW_orig = 0;
FARPROC GetFileVersionInfoSizeA_orig = 0;
FARPROC GetFileVersionInfoSizeExA_orig = 0;
FARPROC GetFileVersionInfoSizeExW_orig = 0;
FARPROC GetFileVersionInfoSizeW_orig = 0;
FARPROC GetFileVersionInfoW_orig = 0;
FARPROC VerFindFileA_orig = 0;
FARPROC VerFindFileW_orig = 0;
FARPROC VerInstallFileA_orig = 0;
FARPROC VerInstallFileW_orig = 0;
FARPROC VerLanguageNameA_orig = 0;
FARPROC VerLanguageNameW_orig = 0;
FARPROC VerQueryValueA_orig = 0;
FARPROC VerQueryValueW_orig = 0;

void GetFileVersionInfoA_new(void);
void GetFileVersionInfoByHandle_new(void);
void GetFileVersionInfoExA_new(void);
void GetFileVersionInfoExW_new(void);
void GetFileVersionInfoSizeA_new(void);
void GetFileVersionInfoSizeExA_new(void);
void GetFileVersionInfoSizeExW_new(void);
void GetFileVersionInfoSizeW_new(void);
void GetFileVersionInfoW_new(void);
void VerFindFileA_new(void);
void VerFindFileW_new(void);
void VerInstallFileA_new(void);
void VerInstallFileW_new(void);
void VerLanguageNameA_new(void);
void VerLanguageNameW_new(void);
void VerQueryValueA_new(void);
void VerQueryValueW_new(void);

BOOL DllInit(void) {
  wchar_t szSystemDirectory[MAX_PATH];
  if (!GetSystemDirectoryW(szSystemDirectory, MAX_PATH)) {
    return FALSE;
  }

  wchar_t szDllPath[MAX_PATH];
  wsprintfW(szDllPath, L"%s/%s", szSystemDirectory, L"version.dll");

  HMODULE hModule = LoadLibraryW(szDllPath);
  if (!hModule) {
    return FALSE;
  }

  GetFileVersionInfoA_orig = GetProcAddress(hModule, "GetFileVersionInfoA");
  GetFileVersionInfoByHandle_orig = GetProcAddress(hModule, "GetFileVersionInfoByHandle");
  GetFileVersionInfoExA_orig = GetProcAddress(hModule, "GetFileVersionInfoExA");
  GetFileVersionInfoExW_orig = GetProcAddress(hModule, "GetFileVersionInfoExW");
  GetFileVersionInfoSizeA_orig = GetProcAddress(hModule, "GetFileVersionInfoSizeA");
  GetFileVersionInfoSizeExA_orig = GetProcAddress(hModule, "GetFileVersionInfoSizeExA");
  GetFileVersionInfoSizeExW_orig = GetProcAddress(hModule, "GetFileVersionInfoSizeExW");
  GetFileVersionInfoSizeW_orig = GetProcAddress(hModule, "GetFileVersionInfoSizeW");
  GetFileVersionInfoW_orig = GetProcAddress(hModule, "GetFileVersionInfoW");
  VerFindFileA_orig = GetProcAddress(hModule, "VerFindFileA");
  VerFindFileW_orig = GetProcAddress(hModule, "VerFindFileW");
  VerInstallFileA_orig = GetProcAddress(hModule, "VerInstallFileA");
  VerInstallFileW_orig = GetProcAddress(hModule, "VerInstallFileW");
  VerLanguageNameA_orig = GetProcAddress(hModule, "VerLanguageNameA");
  VerLanguageNameW_orig = GetProcAddress(hModule, "VerLanguageNameW");
  VerQueryValueA_orig = GetProcAddress(hModule, "VerQueryValueA");
  VerQueryValueW_orig = GetProcAddress(hModule, "VerQueryValueW");

  return TRUE;
}

BOOL APIENTRY DllMain(HMODULE hModule, DWORD dwReason, LPVOID lpReserved) {
  if (dwReason == DLL_PROCESS_ATTACH) {
    if (!DllInit()) {
      MessageBoxW(
        NULL,
        L"DLL initialization failed.",
        L"PlayStation VR2 Toolkit",
        MB_ICONERROR | MB_OK);

      return FALSE;
    }

    return RealDllMain();
  }

  return TRUE;
}
