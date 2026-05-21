#include "server_driver.h"
#include "driver_interface/aston_manager.h"
#include "driver_interface/caesar_manager.h"
#include "driver_interface/config_manager.h"
#include "driver_interface/share_manager.h"
#include "driver_interface/ukf_manager.h"

#include <objbase.h>
#include <timeapi.h>

namespace sie {
namespace psvr2 {

  vr::EVRInitError ServerDriver::Init(vr::IVRDriverContext *pDriverContext) {
    VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

    vr::VRDriverLog()->Log("Activating PlayStation VR2...\n");

    timeBeginPeriod(1);

    CoInitialize(nullptr);

    std::string installPath = vr::VRProperties()->GetStringProperty(pDriverContext->GetDriverHandle(), vr::Prop_InstallPath_String, nullptr);

    ConfigManager::createSingleton();
    ShareManager::createSingleton(1);
    ShareManager::getSingleton()->sub_18015E2A0();
    UkfManager::createSingleton(0);

    vr::VRDriverLog()->Log("Init playstation_vr2_hmd module...\n");

    CaesarManager::createSingleton(installPath.c_str(), 0);

    vr::VRDriverLog()->Log("Init playstation_vr2_hmd module done.\n");

    // Debug data stuff, uses some funky structures. Not required?
    //ShareManager::getSingleton()->sub_18015DFF0();
    //ShareManager::getSingleton()->sub_18015FFA0();

    vr::VRDriverLog()->Log("Init playstation_vr2_sense module...\n");

    AstonManager::createSingleton(installPath.c_str());

    vr::VRDriverLog()->Log("Init playstation_vr2_sense module done.\n");

    // More initialization?

    CheckDirectMode("sie::psvr2::ServerDriver::Init");
  }

  void ServerDriver::Cleanup() {
    // ...
  }

  const char *const *ServerDriver::GetInterfaceVersions() {
    return vr::k_InterfaceVersions;
  }

  void ServerDriver::RunFrame() {
    // ...
  }

  bool ServerDriver::ShouldBlockStandbyMode() {
    return false;
  }

  void ServerDriver::EnterStandby() {
    // ...
  }

  void ServerDriver::LeaveStandby() {
    // ...
  }

  void ServerDriver::CheckDirectMode(const char *callee, bool *out_EdidTypeChanged) {
    char buffer[256];
    bool edidTypeChanged = false;

    bool directModeEnableBefore = vr::VRSettings()->GetBool("direct_mode", "enable");
    float preferredRefreshRate = vr::VRSettings()->GetFloat("steamvr", "preferredRefreshRate");

    m_preferredRefreshRate = preferredRefreshRate;

    if (fabsf(preferredRefreshRate - 90.0f) > 0.1f &&
        fabsf(preferredRefreshRate - 120.0f) > 0.1f)
    {
      m_preferredRefreshRate = 90.0f;
      vr::VRSettings()->RemoveKeyInSection("steamvr", "preferredRefreshRate");
    }

    CaesarManager::EdidType edidType = CaesarManager::EDID_TYPE_90HZ_120HZ;
    CaesarManager::getSingleton()->getEdidType(&edidType);

    if (edidType == CaesarManager::EDID_TYPE_90HZ_ONLY) {
      if (fabsf(m_preferredRefreshRate - 120.0f) < 0.1f) {
        int ret = CaesarManager::getSingleton()->setEdidType(CaesarManager::EDID_TYPE_90HZ_120HZ);

        snprintf(
          buffer,
          sizeof(buffer),
          "setEdidTypeEDID_TYPE_90HZ_120HZ (ret: %d)\n",
          ret
        );
        vr::VRDriverLog()->Log(buffer);

        Sleep(1000);
        edidTypeChanged = true;
      }
    } else if (edidType == CaesarManager::EDID_TYPE_90HZ_120HZ) {
      if (fabsf(m_preferredRefreshRate - 90.0f) < 0.1f) {
        int ret = CaesarManager::getSingleton()->setEdidType(CaesarManager::EDID_TYPE_90HZ_ONLY);

        snprintf(
          buffer,
          sizeof(buffer),
          "EDID_TYPE_90HZ_ONLY (ret: %d)\n",
          ret
        );
        vr::VRDriverLog()->Log(buffer);

        Sleep(1000);
        edidTypeChanged = true;
      }
    }

    snprintf(
      buffer,
      sizeof(buffer),
      "PreferredRefreshRate: %f, EDID TYPE: %d\n",
      m_preferredRefreshRate,
      edidType
    );
    vr::VRDriverLog()->Log(buffer);

    bool directModeEnableAfter = vr::VRSettings()->GetBool("direct_mode", "enable");

    snprintf(
      buffer,
      sizeof(buffer),
      "[DirectMode Check] setting: %d -> %d at %s\n",
      directModeEnableBefore,
      directModeEnableAfter,
      callee
    );
    vr::VRDriverLog()->Log(buffer);

    if (out_EdidTypeChanged)
      *out_EdidTypeChanged = edidTypeChanged;
  }

} // psvr2
} // sie
