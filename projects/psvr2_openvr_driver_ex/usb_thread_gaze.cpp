// #include "common/hmd2_gaze.h" TODO: sort out header re-location
#include "driver_hooks/hmd_device_hooks.h"
#include "driver_interface/caesar_manager.h"
#include "custom_share_manager.h"
#include "usb_thread_gaze.h"
#include "util.h"

#include <openvr_driver.h>
#include <filesystem>
#include <fstream>
#include <vector>


#define GAZE_MAGIC_0 0x47
#define GAZE_MAGIC_1_CAL 0x43
#define GAZE_MAGIC_1_RAW 0x52
#define GAZE_MAGIC_1_STATE 0x53

using namespace psvr2_toolkit;

CaesarUsbThreadGaze *CaesarUsbThreadGaze::m_pInstance = nullptr;

uint8_t CaesarUsbThreadGaze::GetInterface() {
  return 5;
}

uint8_t CaesarUsbThreadGaze::GetEndpoint() {
  return 0x85;
}

void CaesarUsbThreadGaze::OnConnected() {
  vr::ETrackedPropertyError err;
  vr::PropertyContainerHandle_t container = vr::VRDriverHandle();
  uint32_t propSize = vr::VRProperties()->GetStringProperty(container, vr::Prop_UserConfigPath_String, nullptr, 0, &err);
  
  if (propSize > 0) {
    std::string configPath(propSize - 1, '\0');
    vr::VRProperties()->GetStringProperty(container, vr::Prop_UserConfigPath_String, configPath.data(), propSize, &err);

    if (err == vr::TrackedProp_Success) {
      std::filesystem::path dir(configPath);
      std::filesystem::path filePath = dir / "gaze_calibration_blob.bin";

      std::ifstream inFile(filePath, std::ios::binary | std::ios::ate);
      if (inFile.is_open()) {
        std::streamsize size = inFile.tellg();
        inFile.seekg(0, std::ios::beg);

        std::vector<char> buffer(size);
        if (inFile.read(buffer.data(), size)) {
          this->TransferPipe(5, buffer.data(), size);
        }
      }
    }
  }

  // Gaze stream enable. For some reason this doesn't really stick.
  this->ControlCommand(true, 12, nullptr, 0, 0, 0, 1);

  //char data[8] = { 1, 0, 0, 0, 0x05, 0, 0, 0 };
  //CaesarUsbThread::ControlCommandHook(this, true, 0xb, data, 8, 0, 0, 1);
}

int CaesarUsbThreadGaze::PollAndProcess() {
  static hmd2_gaze_status_t state;
  int result = this->TransferPipe(GetEndpoint(), reinterpret_cast<char*>(&state), sizeof(state));

  if (result == 0) {
    // If we timed out, we should try sending the gaze enable again.
    // Entering and exiting passthrough, DP signal changes, and probably some other stuff seems to stop gaze.
    this->ControlCommand(true, 12, nullptr, 0, 0, 0, 1);
    return 0;
  }

  if (result < 0) {
    return -1;
  }
  
  if (state.magic[0] == GAZE_MAGIC_0 && state.magic[1] == GAZE_MAGIC_1_STATE) {
    HmdDeviceHooks::UpdateGaze(&state, sizeof(hmd2_gaze_status_t));
    CustomShareManager* pShareManager = CustomShareManager::getSingleton();
    pShareManager->setGazeStatus(&state);
  }

  return 0;
}
