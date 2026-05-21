#include "device_provider_proxy.h"
#include "hmd_driver_loader.h"

#include "server_driver.h"

#include <openvr_driver.h>
#include <windows.h>

using namespace psvr2_toolkit;

sie::psvr2::ServerDriver serverDriver;

extern "C" __declspec(dllexport) void *HmdDriverFactory(const char *pInterfaceName, int *pReturnCode) {
  static HmdDriverLoader *pHmdDriverLoader = HmdDriverLoader::Instance();

  // Check if the HMD driver DLL is actually loaded.
  if (pHmdDriverLoader->GetBaseAddress()) {
    if (strcmp(vr::IServerTrackedDeviceProvider_Version, pInterfaceName) == 0) {
      return &serverDriver;
    }
  }

  MessageBoxW(nullptr, L"Loading original HMD driver failed, please report this to the developers!", L"PlayStation VR2 Toolkit", MB_ICONERROR | MB_OK);

  if (pReturnCode) {
    *pReturnCode = vr::VRInitError_Init_InterfaceNotFound;
  }

  return nullptr;
}
