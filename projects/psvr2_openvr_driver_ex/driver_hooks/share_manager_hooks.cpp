#include "share_manager_hooks.h"
#include "../hmd_driver_loader.h"
#include "../hook_lib.h"

namespace psvr2_toolkit {

  void ShareManager__createSingletonHook(int a1) {
    static HmdDriverLoader *pHmdDriverLoader = HmdDriverLoader::Instance();

    *reinterpret_cast<bool *>(pHmdDriverLoader->GetBaseAddress() + 0x354F80) = true; // ShareManager::m_initialized
    *reinterpret_cast<uintptr_t *>(pHmdDriverLoader->GetBaseAddress() + 0x354F78) = 0xCAFEBABE; // ShareManager::m_pInstance
  }

  uintptr_t ShareManager__getSingletonHook() {
    return 0xCAFEBABE;
  }

  void ShareManager__sub_18015E2A0Hook(void *thisptr) {
    // This function is run on startup, it sets a DWORD to 0.
    // TODO: Reverse this.
  }

  // Hopefully no register stomping...
  void ShareManager__sub_18015E990Hook(void *thisptr, void* a2) {
    // This function is run inside CaesarUsbThreadImuStatus::initialize, it appears to copy a buffer.
    // TODO: Reverse this.
  }

  void ShareManager__sub_18015F330Hook(void *thisptr, void* a2) {
    // This function is run inside CaesarUsbThreadImuStatus::initialize, it sets a few variables.
    // TODO: Reverse this.
  }

  // Some config thing.
  void ShareManager__sub_18015B950Hook(void *thisptr, int configId, int *outval) {
    // This function is run inside CaesarUsbThreadImuStatus::initialize, it gets a config variable.
    // TODO: Reverse this.
  }

  void ShareManagerHooks::InstallHooks() {
    static HmdDriverLoader *pHmdDriverLoader = HmdDriverLoader::Instance();

    // ShareManager::createSingleton
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15BCF0),
                         reinterpret_cast<void *>(ShareManager__createSingletonHook));

    // ShareManager::getSingleton
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15BBD0),
                         reinterpret_cast<void *>(ShareManager__getSingletonHook));

    // ShareManager::sub_18015E2A0
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15E2A0),
                         reinterpret_cast<void *>(ShareManager__sub_18015E2A0Hook));

    // ShareManager::sub_18015E990
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15E990),
                         reinterpret_cast<void *>(ShareManager__sub_18015E990Hook));

    // ShareManager::sub_18015F330
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15F330),
                         reinterpret_cast<void *>(ShareManager__sub_18015F330Hook));

    // ShareManager::sub_18015B950
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15B950),
                         reinterpret_cast<void *>(ShareManager__sub_18015B950Hook));
  }

}
