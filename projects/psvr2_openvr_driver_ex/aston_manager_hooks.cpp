#include "aston_manager_hooks.h"

#include "hmd_driver_loader.h"
#include "hook_lib.h"
#include "util.h"

namespace psvr2_toolkit {

  void *(*AstonManager__acquireLibpadAccess)(void *) = nullptr;
  void AstonManager__acquireLibpadAccessHook(void *thisptr) {
    static bool ranOnce = false;

    // For some reason, a deadlock occurs when this function is ran twice under Ignition.
    // So, we'll just only allow this to run once.
    if (!ranOnce) {
      AstonManager__acquireLibpadAccess(thisptr);
      ranOnce = true;
    }
  }

  void AstonManagerHooks::InstallHooks() {
    static HmdDriverLoader *pHmdDriverLoader = HmdDriverLoader::Instance();

    // AstonManager::acquireLibpadAccess
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x11d0c0),
                         reinterpret_cast<void *>(AstonManager__acquireLibpadAccessHook),
                         reinterpret_cast<void **>(&AstonManager__acquireLibpadAccess));

    // Controller poll rate stub
    if (Util::IsRunningOnWine()) {
      Util::DriverLog("Stubbing controller poll rate update due to Bluetooth limitations.");
      HookLib::InstallStub(reinterpret_cast<void*>(pHmdDriverLoader->GetBaseAddress() + 0x1cdff0));
    }
  }

} // psvr2_toolkit
