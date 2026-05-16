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

  void ShareManager__destroySingletonHook() {
    // ...
  }

  void ShareManager__sub_18015E2A0Hook(void *thisptr) {
    // This function is run on startup, it sets a DWORD to 0.
    // Appears to use mutex/event SHARE_VRT2_WIN_TELEMETRY_DEV_INFO.
    // TODO: Reverse this.
  }

  // Hopefully no register stomping...
  void ShareManager__setCalibDataHook(void *thisptr, uint8_t *calibData) {
    // This function is run inside CaesarUsbThreadImuStatus::initialize, it appears to copy a buffer.
    // Uses mutex/event SHARE_VRT2_WIN_CALIB.
    // TODO: Reverse this.
  }

  void ShareManager__setFwInfoHook(void *thisptr, void* a2) {
    // This function is run inside CaesarUsbThreadImuStatus::initialize, it sets a few variables.
    // Uses mutex/event SHARE_VRT2_WIN_FW_INFO.
    // TODO: Reverse this.
  }

  // Some config thing.
  void ShareManager__sub_18015B950Hook(void *thisptr, int configId, int *outval) {
    // This function is run inside CaesarUsbThreadImuStatus::initialize, it gets a config variable.
    // TODO: Reverse this.
  }

  void ShareManager__sub_18015DFF0Hook(void *thisptr, void* a2) {
    // This function is run inside multiple places, it appears to copy a buffer.
    // Uses mutex/event SHARE_VRT2_WIN_DEBUG_DATA.
    // TODO: Reverse this.
  }

  void ShareManager__sub_18015FFA0Hook(void *thisptr, void* a2) {
    // This function is run inside multiple places, it appears to copy a buffer.
    // Uses mutex/event SHARE_VRT2_WIN_DEBUG_DATA.
    // TODO: Reverse this.
  }

  void ShareManager__sub_18015F850Hook(void *thisptr, void* a2) {
    // This function is run inside multiple places, it sets a few variables.
    // Uses mutex/event SHARE_VRT2_WIN_IR_CAM_SETTING.
    // TODO: Reverse this.
  }

  void ShareManager__sub_18015CFD0Hook(void *thisptr, void* a2) {
    // This function is run inside multiple places, it appears to copy a buffer.
    // Uses mutex/event SHARE_VRT2_WIN_CONT_CONFIG.
    // TODO: Reverse this.
  }

  void ShareManager__sub_18015F110Hook(void *thisptr, void* a2) {
    // This function is run inside sub_18018A7E0, it appears to copy a buffer.
    // Uses mutex/event SHARE_VRT2_WIN_CONT_CONFIG.
    // TODO: Reverse this.
  }

  void ShareManager__sub_18015F6F0Hook(void *thisptr, void* a2) {
    // This function is run inside multiple places, it sets a few variables.
    // Uses mutex/event SHARE_VRT2_WIN_IMAGE_SETTING.
    // TODO: Reverse this.
  }

  void ShareManager__sub_18015D6B0Hook(void *thisptr, void* a2, void* a3) {
    // This function is run inside multiple places, it gets a few variables.
    // Uses mutex/event SHARE_VRT2_WIN_IR_CAM_SETTING.
    // TODO: Reverse this.
  }

  void ShareManager__sub_18015C9D0Hook(void *thisptr, void* a2, void* a3) {
    // This function is run inside multiple places, it gets a few variables.
    // Uses mutex/event SHARE_VRT2_WIN_BLOB_CONFIG.
    // TODO: Reverse this.
  }

  void ShareManager__sub_18015D550Hook(void *thisptr, void* a2, void* a3) {
    // This function is run inside multiple places, it gets a few variables.
    // Uses mutex/event SHARE_VRT2_WIN_IMAGE_SETTING.
    // TODO: Reverse this.
  }

  void ShareManager__sub_18015EB70Hook(void *thisptr, void* a2) {
    // This function is run inside sub_180127250, it gets a few variables.
    // Uses mutex/event SHARE_VRT2_WIN_CALIB.
    // TODO: Reverse this.
  }

  void ShareManager__sub_18015FC00Hook(void *thisptr, void* a2) {
    // This function is run inside sub_180128240, it gets a few variables(?)
    // Uses mutex/event SHARE_VRT2_WIN_STATUS.
    // TODO: Reverse this.
  }

  // Some config thing.
  void ShareManager__getConfigIntValueHook(void *thisptr, int configId, int *outval) {
    // TODO: Reverse this.
  }

  void ShareManager__sub_18015EA40Hook(void *thisptr, void* a2) {
    // This function is run inside sub_180127250, it gets/sets a few variables.
    // Uses mutex/event SHARE_VRT2_WIN_CALIB.
    // TODO: Reverse this.
  }

  void ShareManager__sub_18015C7E0Hook(void *thisptr, void* a2) {
    // This function is run inside multiple places, it gets/sets a few variables.
    // Uses mutex/event SHARE_VRT2_WIN_COMMON.
    // TODO: Reverse this.
  }

  void ShareManager__sub_18015E0B0Hook(void *thisptr, void* a2, void* a3) {
    // This function is run inside multiple places, it does some weird stuff.
    // TODO: Reverse this.
  }

  // my ass hurts, i've been sitting in my chair for far too long writing these hooks.
  // save me please, i'll do anything.

  void ShareManager__sub_18015EB00Hook(void *thisptr, void* a2) {
    // This function is run inside sub_180127250, it does some weird stuff.
    // Uses mutex/event SHARE_VRT2_WIN_CALIB.
    // TODO: Reverse this.
  }

  // PLEASE, HOW MANY MORE ACCESS VIOLATIONS IS THERE UNTIL THE DRIVER AT LEAST STARTS.

  void ShareManager__sub_18015F610Hook(void *thisptr, void* a2) {
    // This function is run inside sub_180127250, it does some weird stuff.
    // Uses mutex/event SHARE_VRT2_WIN_CALIB.
    // TODO: Reverse this.
  }

  void ShareManager__sub_18015DBA0Hook(void *thisptr, void* a2) {
    // This function is run inside multiple places, it does some weird stuff.
    // Uses mutex/event SHARE_VRT2_WIN_TELEMETRY_TRACKING_INFO.
    // TODO: Reverse this.
  }

  void ShareManager__sub_18015E850Hook(void *thisptr, void* a2) {
    // This function is run inside sub_180127250, it does some weird stuff.
    // Uses mutex/event SHARE_VRT2_WIN_CALIB.
    // TODO: Reverse this.
  }

  void ShareManager__sub_18015FC70Hook(void *thisptr, void* a2) {
    // This function is run inside multiple places, it does some weird stuff.
    // Uses mutex/event SHARE_VRT2_WIN_TELEMETRY_TRACKING_INFO.
    // TODO: Reverse this.
  }

  void ShareManager__sub_18015E910Hook(void *thisptr, void* a2) {
    // This function is run inside sub_180127250, it does some weird stuff.
    // Uses mutex/event SHARE_VRT2_WIN_CALIB.
    // TODO: Reverse this.
  }

  void ShareManager__sub_18015F9D0Hook(void *thisptr, void* a2) {
    // This function is run inside sub_18011FC50, it does some weird stuff.
    // Uses mutex/event SHARE_VRT2_WIN_VR_DIALOG.
    // TODO: Reverse this.
  }

  void ShareManager__sub_18015E4A0Hook(void *thisptr, int a2, int a3) {
    // This function is run inside multiple places, it does some weird stuff.
    // TODO: Reverse this.
  }

  void ShareManager__sub_18015D730Hook(void *thisptr, void* a2, int a3) {
    // This function is run inside multiple places, it does some weird stuff.
    // Uses mutex/event SHARE_VRT2_WIN_TELEMETRY_DEV_INFO.
    // TODO: Reverse this.
  }

  void ShareManager__sub_18015A250Hook(void *thisptr, int a2, void* a3) {
    // This function is run inside multiple places, it appears to write the pose into share buffer?
    // Appears to use mutex/event SHARE_VRT2_WIN_POSE_HMD/SHARE_VRT2_WIN_POSE_CONT_(R/L)?
    // TODO: Reverse this.
  }

  void ShareManager__sub_18015E260Hook(void *thisptr, int a2) {
    // This function is run inside sub_18011D0C0, it appears to try acquiring libpad access?
    // Appears to use mutex/event SHARE_VRT2_WIN_LIBPAD_ACCESS?
    // TODO: Reverse this.
  }

  bool ShareManager__sub_18015C610Hook(void *thisptr, int a2) {
    // This function is run inside sub_18011BAE0, it does some more libpad stuff...
    // TODO: Reverse this.
    return false; // It appears returning false will prevent libpad from opening device.
  }

  bool ShareManager__sub_180159940Hook(void *thisptr) {
    // This function is run inside sub_18011D0C0, it appears to try acquiring libpad access?
    // Appears to use mutex/event SHARE_VRT2_WIN_LIBPAD_REQUEST_STEAM_VR_PLUGIN?
    // TODO: Reverse this.
    return true; // It appears returning true will prevent libpad from opening device.
  }

  void ShareManager__sub_18015F1D0Hook(void *thisptr, void* a2) {
    // This function is run inside sub_1801617C0, it appears to try logging stuff?
    // Appears to use mutex/event SHARE_VRT2_WIN_CONT_LED_INFO?
    // TODO: Reverse this.
  }

  void ShareManager__sub_18015DB30Hook(void *thisptr, void* a2) {
    // This function is run inside sub_180132890, no clue what it does?
    // Appears to use mutex/event SHARE_VRT2_WIN_STATUS?
    // TODO: Reverse this.
  }

  void ShareManager__sub_18015DF60Hook(void *thisptr, void* a2) {
    // This function is run inside sub_180132890, no clue what it does?
    // Appears to use mutex/event SHARE_VRT2_WIN_APPLICATION?
    // TODO: Reverse this.
  }

  void ShareManager__sub_18015FF00Hook(void *thisptr, void* a2) {
    // This function is run inside sub_180132890, no clue what it does?
    // Appears to use mutex/event SHARE_VRT2_WIN_APPLICATION?
    // TODO: Reverse this.
  }

  void ShareManager__sub_18015DF00Hook(void *thisptr, void* a2) {
    // This function is run inside sub_180133960, no clue what it does?
    // Appears to use mutex/event SHARE_VRT2_WIN_INITIAL_SETUP_INFO?
    // TODO: Reverse this.
  }

  void ShareManager__sub_18015CA50Hook(void *thisptr, void* a2, int a3) {
    // This function is run inside sub_180133960, no clue what it does?
    // Appears to use mutex/event SHARE_VRT2_WIN_BLUETOOTH_QUALITY_INFO?
    // TODO: Reverse this.
  }

  void ShareManager__sub_18015FA80Hook(void *thisptr, void* a2) {
    // This function is run inside sub_1801AC4E0, no clue what it does?
    // Appears to use mutex/event SHARE_VRT2_WIN_PLAYAREA_RESULT?
    // TODO: Reverse this.
  }

  void ShareManager__sub_18015F470Hook(void *thisptr, int a2, void* a3) {
    // This function is run inside multiple places, appears to write to config?
    // TODO: Reverse this.
  }

  int ShareManager__sub_180159DC0Hook(void *thisptr, int a2, void* a3, void* a4) {
    // This function is run inside sub_18011F1B0, no clue what it does?
    // TODO: Reverse this.
    return -1;
  }

  void ShareManagerHooks::InstallHooks() {
    static HmdDriverLoader *pHmdDriverLoader = HmdDriverLoader::Instance();

    // TODO: We really should consider making this a macro...

    // ShareManager::createSingleton
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15BCF0),
                         reinterpret_cast<void *>(ShareManager__createSingletonHook));

    // ShareManager::getSingleton
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15BBD0),
                         reinterpret_cast<void *>(ShareManager__getSingletonHook));

    // ShareManager::destroySingleton
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15E360),
                         reinterpret_cast<void *>(ShareManager__destroySingletonHook));

    // ShareManager::sub_18015E2A0
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15E2A0),
                         reinterpret_cast<void *>(ShareManager__sub_18015E2A0Hook));

    // ShareManager::setCalibData
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15E990),
                         reinterpret_cast<void *>(ShareManager__setCalibDataHook));

    // ShareManager::setFwInfo
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15F330),
                         reinterpret_cast<void *>(ShareManager__setFwInfoHook));

    // ShareManager::sub_18015B950
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15B950),
                         reinterpret_cast<void *>(ShareManager__sub_18015B950Hook));

    // ShareManager::sub_18015DFF0
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15DFF0),
                         reinterpret_cast<void *>(ShareManager__sub_18015DFF0Hook));

    // ShareManager::sub_18015FFA0
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15FFA0),
                         reinterpret_cast<void *>(ShareManager__sub_18015FFA0Hook));

    // ShareManager::sub_18015F850
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15F850),
                         reinterpret_cast<void *>(ShareManager__sub_18015F850Hook));

    // ShareManager::sub_18015CFD0
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15CFD0),
                         reinterpret_cast<void *>(ShareManager__sub_18015CFD0Hook));

    // ShareManager::sub_18015F110
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15F110),
                         reinterpret_cast<void *>(ShareManager__sub_18015F110Hook));

    // ShareManager::sub_18015F6F0
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15F6F0),
                         reinterpret_cast<void *>(ShareManager__sub_18015F6F0Hook));

    // ShareManager::sub_18015D6B0
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15D6B0),
                         reinterpret_cast<void *>(ShareManager__sub_18015D6B0Hook));

    // ShareManager::sub_18015C9D0
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15C9D0),
                         reinterpret_cast<void *>(ShareManager__sub_18015C9D0Hook));

    // ShareManager::sub_18015D550
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15D550),
                         reinterpret_cast<void *>(ShareManager__sub_18015D550Hook));

    // ShareManager::sub_18015EB70
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15EB70),
                         reinterpret_cast<void *>(ShareManager__sub_18015EB70Hook));

    // ShareManager::sub_18015FC00
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15FC00),
                         reinterpret_cast<void *>(ShareManager__sub_18015FC00Hook));

    // ShareManager::getConfigIntValue
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15D270),
                         reinterpret_cast<void *>(ShareManager__getConfigIntValueHook));

    // ShareManager::sub_18015EA40
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15EA40),
                         reinterpret_cast<void *>(ShareManager__sub_18015EA40Hook));

    // ShareManager::sub_18015C7E0
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15C7E0),
                         reinterpret_cast<void *>(ShareManager__sub_18015C7E0Hook));

    // ShareManager::sub_18015E0B0
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15E0B0),
                         reinterpret_cast<void *>(ShareManager__sub_18015E0B0Hook));

    // ShareManager::sub_18015EB00
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15EB00),
                         reinterpret_cast<void *>(ShareManager__sub_18015EB00Hook));

    // ShareManager::sub_18015F610
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15F610),
                         reinterpret_cast<void *>(ShareManager__sub_18015F610Hook));

    // ShareManager::sub_18015DBA0
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15DBA0),
                         reinterpret_cast<void *>(ShareManager__sub_18015DBA0Hook));

    // ShareManager::sub_18015E850
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15E850),
                         reinterpret_cast<void *>(ShareManager__sub_18015E850Hook));

    // ShareManager::sub_18015FC70
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15FC70),
                         reinterpret_cast<void *>(ShareManager__sub_18015FC70Hook));

    // ShareManager::sub_18015E910
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15E910),
                         reinterpret_cast<void *>(ShareManager__sub_18015E910Hook));

    // ShareManager::sub_18015F9D0
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15F9D0),
                         reinterpret_cast<void *>(ShareManager__sub_18015F9D0Hook));

    // ShareManager::sub_18015E4A0
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15E4A0),
                         reinterpret_cast<void *>(ShareManager__sub_18015E4A0Hook));

    // ShareManager::sub_18015D730
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15D730),
                         reinterpret_cast<void *>(ShareManager__sub_18015D730Hook));

    // ShareManager::sub_18015A250
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15A250),
                         reinterpret_cast<void *>(ShareManager__sub_18015A250Hook));

    // ShareManager::sub_18015E260
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15E260),
                         reinterpret_cast<void *>(ShareManager__sub_18015E260Hook));

    // ShareManager::sub_18015C610
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15C610),
                         reinterpret_cast<void *>(ShareManager__sub_18015C610Hook));

    // ShareManager::sub_180159940
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x159940),
                         reinterpret_cast<void *>(ShareManager__sub_180159940Hook));

    // ShareManager::sub_18015F1D0
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15F1D0),
                         reinterpret_cast<void *>(ShareManager__sub_18015F1D0Hook));

    // ShareManager::sub_18015DB30
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15DB30),
                         reinterpret_cast<void *>(ShareManager__sub_18015DB30Hook));

    // ShareManager::sub_18015DF60
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15DF60),
                         reinterpret_cast<void *>(ShareManager__sub_18015DF60Hook));

    // ShareManager::sub_18015FF00
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15FF00),
                         reinterpret_cast<void *>(ShareManager__sub_18015FF00Hook));

    // ShareManager::sub_18015DF00
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15DF00),
                         reinterpret_cast<void *>(ShareManager__sub_18015DF00Hook));

    // ShareManager::sub_18015CA50
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15CA50),
                         reinterpret_cast<void *>(ShareManager__sub_18015CA50Hook));

    // ShareManager::sub_18015FA80
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15FA80),
                         reinterpret_cast<void *>(ShareManager__sub_18015FA80Hook));

    // ShareManager::sub_18015F470
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x15F470),
                         reinterpret_cast<void *>(ShareManager__sub_18015F470Hook));

    // ShareManager::sub_180159DC0
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x159DC0),
                         reinterpret_cast<void *>(ShareManager__sub_180159DC0Hook));
  }

}
