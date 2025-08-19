#include "device_provider_proxy.h"
#include "hmd_driver_loader.h"

#include <openvr_driver.h>
#include <windows.h>

using namespace psvr2_toolkit;

extern "C" __declspec(dllexport) void *HmdDriverFactory(const char *pInterfaceName, int *pReturnCode) {
  static HmdDriverLoader *pHmdDriverLoader = HmdDriverLoader::Instance();

  // Check if the HMD driver DLL is actually loaded.
  if (pHmdDriverLoader->GetHandle()) {
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
