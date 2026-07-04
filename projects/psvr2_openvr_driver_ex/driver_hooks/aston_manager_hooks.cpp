#include "aston_manager_hooks.h"

#include "hmd_driver_loader.h"
#include "hook_lib.h"
#include "util.h"

namespace psvr2_toolkit {
  void AstonManagerHooks::InstallHooks() {
    static HmdDriverLoader *pHmdDriverLoader = HmdDriverLoader::Instance();

    // Controller poll rate stub
    if (Util::IsRunningOnWine()) {
      Util::DriverLog("Stubbing controller poll rate update due to Bluetooth limitations.");
      HookLib::InstallStub(reinterpret_cast<void*>(pHmdDriverLoader->GetBaseAddress() + 0x1cdff0));
    }
  }

} // psvr2_toolkit
