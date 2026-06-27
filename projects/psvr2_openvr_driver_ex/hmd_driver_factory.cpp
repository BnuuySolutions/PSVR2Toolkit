#include "device_provider_proxy.h"
#include "hmd_driver_loader.h"

#include <openvr_driver.h>
#include <windows.h>
#include <string>

using namespace psvr2_toolkit;

extern "C" __declspec(dllexport) void *HmdDriverFactory(const char *pInterfaceName, int *pReturnCode) {
  static HmdDriverLoader *pHmdDriverLoader = HmdDriverLoader::Instance();

  // Check if the HMD driver DLL is actually loaded.
  if (pHmdDriverLoader->GetBaseAddress()) {
    void *result = pHmdDriverLoader->pfnHmdDriverFactory(pInterfaceName, pReturnCode);

    if (strcmp(vr::IServerTrackedDeviceProvider_Version, pInterfaceName) == 0 &&
        *pReturnCode == vr::VRInitError_None /* no point injecting if the original driver failed to initialise */) {
      static DeviceProviderProxy *pDeviceProviderProxy = DeviceProviderProxy::Instance();
      pDeviceProviderProxy->SetDeviceProvider(static_cast<vr::IServerTrackedDeviceProvider *>(result));
      return pDeviceProviderProxy;
    }

    return result;
  }

  MessageBoxW(nullptr, L"Loading original HMD driver failed, please report this to the developers!", L"PlayStation VR2 Toolkit (DriverEx)", MB_ICONERROR | MB_OK);

  if (pReturnCode) {
    *pReturnCode = vr::VRInitError_Init_InterfaceNotFound;
  }

  return nullptr;
}

BOOL APIENTRY DllMain(HMODULE hModule,
                      DWORD ul_reason_for_call,
                      LPVOID lpReserved)
{
    switch (ul_reason_for_call)
    {
    case DLL_PROCESS_ATTACH:
    {
        wchar_t path[MAX_PATH];
        GetModuleFileNameW(nullptr, path, MAX_PATH);
        const wchar_t* processName = wcsrchr(path, L'\\');
        processName = processName ? processName + 1 : path;
        std::wstring message = L"Process:\n";
        message += processName;
        MessageBoxW(
            nullptr,
            message.c_str(),
            L"PlayStation VR2 Toolkit (DriverEx)",
            MB_ICONINFORMATION | MB_OK
        );
        break;
    }
    }

    return TRUE;
}
