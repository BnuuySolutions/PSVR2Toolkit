#include "server_driver.h"
#include "driver_interface/aston_manager.h"
#include "driver_interface/caesar_manager.h"

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

    vr::VRDriverLog()->Log("Init playstation_vr2_hmd module...\n");

    CaesarManager::createSingleton(installPath.c_str(), 0);

    vr::VRDriverLog()->Log("Init playstation_vr2_hmd module done.\n");

    vr::VRDriverLog()->Log("Init playstation_vr2_sense module...\n");

    AstonManager::createSingleton(installPath.c_str());

    vr::VRDriverLog()->Log("Init playstation_vr2_sense module done.\n");
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

} // psvr2
} // sie
