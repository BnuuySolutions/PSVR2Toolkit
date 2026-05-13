#include "share_manager_hooks.h"
#include "../hmd_driver_loader.h"
#include "../hook_lib.h"

namespace psvr2_toolkit {

  void ShareManager__createSingletonHook(int a1) {
    static HmdDriverLoader *pHmdDriverLoader = HmdDriverLoader::Instance();

    *reinterpret_cast<bool*>(pHmdDriverLoader->GetBaseAddress() + 0x354F80) = true; // ShareManager::m_initialized
    *reinterpret_cast<uintptr_t*>(pHmdDriverLoader->GetBaseAddress() + 0x354F78) = 0xCAFEBABE; // ShareManager::m_pInstance
  }

  void ShareManagerHooks::InstallHooks() {
    static HmdDriverLoader *pHmdDriverLoader = HmdDriverLoader::Instance();

    // ShareManager::createSingleton
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15BCF0),
                         reinterpret_cast<void *>(ShareManager__createSingletonHook));
  }

}
