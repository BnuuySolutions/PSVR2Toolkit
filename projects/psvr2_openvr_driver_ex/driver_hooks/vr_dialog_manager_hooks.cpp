#include "vr_dialog_manager_hooks.h"

#include "hmd_driver_loader.h"
#include "utils/hook_lib.h"
#include "util.h"

#include <openvr_driver.h>

namespace psvr2_toolkit {

static constexpr const char *kBackupSection = "playstation_vr2_ex_collisionBounds_Backup";

static bool HasBackupSettings() {
  vr::IVRSettings *pSettings = vr::VRSettings();
  if (!pSettings) {
    return false;
  }

  vr::EVRSettingsError error;
  pSettings->GetFloat(kBackupSection, vr::k_pch_CollisionBounds_FadeDistance_Float, &error);
  return error == vr::EVRSettingsError::VRSettingsError_None;
}

static void sie__psvr2__VrDialogManager__hideChaperone_Hook(void *thisptr) {
  if (HasBackupSettings()) {
    return;
  }

  vr::IVRSettings *pSettings = vr::VRSettings();
  if (pSettings) {
    float fadeDistance = pSettings->GetFloat(vr::k_pch_CollisionBounds_Section, vr::k_pch_CollisionBounds_FadeDistance_Float);
    bool playSpaceOn = pSettings->GetBool(vr::k_pch_CollisionBounds_Section, vr::k_pch_CollisionBounds_PlaySpaceOn_Bool);
    bool groundPerimeterOn = pSettings->GetBool(vr::k_pch_CollisionBounds_Section, vr::k_pch_CollisionBounds_GroundPerimeterOn_Bool);
    bool centerMarkerOn = pSettings->GetBool(vr::k_pch_CollisionBounds_Section, vr::k_pch_CollisionBounds_CenterMarkerOn_Bool);

    pSettings->SetFloat(kBackupSection, vr::k_pch_CollisionBounds_FadeDistance_Float, fadeDistance);
    pSettings->SetBool(kBackupSection, vr::k_pch_CollisionBounds_PlaySpaceOn_Bool, playSpaceOn);
    pSettings->SetBool(kBackupSection, vr::k_pch_CollisionBounds_GroundPerimeterOn_Bool, groundPerimeterOn);
    pSettings->SetBool(kBackupSection, vr::k_pch_CollisionBounds_CenterMarkerOn_Bool, centerMarkerOn);

    pSettings->SetFloat(vr::k_pch_CollisionBounds_Section, vr::k_pch_CollisionBounds_FadeDistance_Float, 0.0f);
    pSettings->SetBool(vr::k_pch_CollisionBounds_Section, vr::k_pch_CollisionBounds_PlaySpaceOn_Bool, false);
    pSettings->SetBool(vr::k_pch_CollisionBounds_Section, vr::k_pch_CollisionBounds_GroundPerimeterOn_Bool, false);
    pSettings->SetBool(vr::k_pch_CollisionBounds_Section, vr::k_pch_CollisionBounds_CenterMarkerOn_Bool, false);

    Util::DriverLog("[VrTracker2] hide Chaperone (backed up FadeDistance: {})\n", fadeDistance);
  }
}

static void sie__psvr2__VrDialogManager__showChaperone_Hook(void *thisptr) {
  if (!HasBackupSettings()) {
    return;
  }

  vr::IVRSettings *pSettings = vr::VRSettings();
  if (pSettings) {
    float fadeDistance = pSettings->GetFloat(kBackupSection, vr::k_pch_CollisionBounds_FadeDistance_Float);
    bool playSpaceOn = pSettings->GetBool(kBackupSection, vr::k_pch_CollisionBounds_PlaySpaceOn_Bool);
    bool groundPerimeterOn = pSettings->GetBool(kBackupSection, vr::k_pch_CollisionBounds_GroundPerimeterOn_Bool);
    bool centerMarkerOn = pSettings->GetBool(kBackupSection, vr::k_pch_CollisionBounds_CenterMarkerOn_Bool);

    pSettings->SetFloat(vr::k_pch_CollisionBounds_Section, vr::k_pch_CollisionBounds_FadeDistance_Float, fadeDistance);
    pSettings->SetBool(vr::k_pch_CollisionBounds_Section, vr::k_pch_CollisionBounds_PlaySpaceOn_Bool, playSpaceOn);
    pSettings->SetBool(vr::k_pch_CollisionBounds_Section, vr::k_pch_CollisionBounds_GroundPerimeterOn_Bool, groundPerimeterOn);
    pSettings->SetBool(vr::k_pch_CollisionBounds_Section, vr::k_pch_CollisionBounds_CenterMarkerOn_Bool, centerMarkerOn);

    pSettings->RemoveSection(kBackupSection);

    Util::DriverLog("[VrTracker2] show Chaperone (restored FadeDistance: {})\n", fadeDistance);
  }
}

void VrDialogManagerHooks::InstallHooks() {
  static HmdDriverLoader *pHmdDriverLoader = HmdDriverLoader::Instance();

  HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x131390),
                       reinterpret_cast<void *>(sie__psvr2__VrDialogManager__hideChaperone_Hook));

  HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x133CA0),
                       reinterpret_cast<void *>(sie__psvr2__VrDialogManager__showChaperone_Hook));

  // Remove signature checks.
  INSTALL_STUB_RET0(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x134FF0)); // VrDialogManager::VerifyLibrary

  // Remove dashboard, dialog, and desktop app process launch.
  INSTALL_STUB(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x12F830)); // VrDialogManager::CreateDashboardProcess
  INSTALL_STUB(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x130020)); // VrDialogManager::CreateDialogProcess
  INSTALL_STUB(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x131D90)); // VrDialogManager::CreateDesktopAppProcess
}

} // namespace psvr2_toolkit
