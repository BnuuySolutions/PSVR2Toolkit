#include "driver_hooks/hmd_device_hooks.h"
#include "driver_interface/caesar_manager.h"
#include "custom_share_manager.h"
#include "hmd2_gen_data.h"
#include "usb_thread_gaze.h"
#include "util.h"

#include <openvr_driver.h>
#include <filesystem>
#include <fstream>
#include <vector>

#define GAZE_MAGIC_0 'G'
#define GAZE_MAGIC_1_CAL 'C'
#define GAZE_MAGIC_1_RAW 'R'
#define GAZE_MAGIC_1_STATE 'S'

using namespace psvr2_toolkit;

CaesarUsbThreadGaze *CaesarUsbThreadGaze::m_pInstance = nullptr;

uint8_t CaesarUsbThreadGaze::GetInterface() { return 5; }

uint8_t CaesarUsbThreadGaze::GetEndpoint() { return 0x85; }

void CaesarUsbThreadGaze::OnConnected() {
  vr::ETrackedPropertyError err;
  vr::PropertyContainerHandle_t container = vr::VRDriverHandle();
  uint32_t propSize = vr::VRProperties()->GetStringProperty(container, vr::Prop_UserConfigPath_String, nullptr, 0, &err);

  if (propSize > 0) {
    std::string configPath(propSize - 1, '\0');
    vr::VRProperties()->GetStringProperty(container, vr::Prop_UserConfigPath_String, configPath.data(), propSize, &err);

    if (err == vr::TrackedProp_Success) {
      std::filesystem::path dir(configPath);
      std::filesystem::path filePath = dir / HMD2_GAZE_CALIB_BLOB_FILENAME;

      std::ifstream inFile(filePath, std::ios::binary | std::ios::ate);
      if (inFile.is_open()) {
        std::streamsize size = inFile.tellg();
        inFile.seekg(0, std::ios::beg);

        // Nothing validated this file before uploading it to the headset: no size cap, no header check,
        // and no check of the transfer result. A truncated or corrupt blob could be sent straight in.
        if (size < static_cast<std::streamsize>(sizeof(Hmd2GazeCalibHeader)) || size > static_cast<std::streamsize>(k_hmd2GazeCalibMaxBlobSize)) {
          Util::DriverLog("[Gaze] Calibration blob size {} is out of range; ignoring it.", static_cast<int64_t>(size));
        } else {
          std::vector<char> buffer(static_cast<size_t>(size));
          if (!inFile.read(buffer.data(), size)) {
            Util::DriverLog("[Gaze] Could not read the calibration blob.");
          } else if (!Hmd2IsGazeCalibBlobValid(buffer.data(), static_cast<uint32_t>(size))) {
            Util::DriverLog("[Gaze] Calibration blob failed validation; ignoring it.");
          } else if (this->TransferPipe(5, buffer.data(), buffer.size()) < 0) {
            Util::DriverLog("[Gaze] Failed to upload the calibration blob to the headset.");
          } else {
            Util::DriverLog("[Gaze] Uploaded calibration blob ({} bytes).", static_cast<int64_t>(size));
          }
        }
      }
    }
  }

  // Gaze stream enable. For some reason this doesn't really stick.
  this->ControlCommand(true, 0x0C, nullptr, 0, 0, 0, 1);
}

int CaesarUsbThreadGaze::PollAndProcess() {
  static constexpr size_t k_gazeStatusSize = sizeof(hmd2_gaze_status_t);

  // Deliberately not static: a partial transfer would otherwise leave the untouched tail holding the
  // previous frame, which is exactly what makes a short read look like a valid sample.
  hmd2_gaze_status_t state{};
  int result = this->TransferPipe(GetEndpoint(), reinterpret_cast<char *>(&state), sizeof(state), 500);

  if (result == 0) {
    // If we timed out, we should try sending the gaze enable again.
    // Entering and exiting passthrough, DP signal changes, and probably some other stuff seems to stop gaze.
    this->ControlCommand(true, 0x0C, nullptr, 0, 0, 0, 1);
    return 0;
  }

  if (result < 0) {
    return -1;
  }

  // TransferPipe returns the byte count, and a timed-out-but-partial transfer reports a positive value
  // smaller than the struct. Publishing that would hand downstream a half-filled frame carrying a valid
  // magic and a plausible timestamp, which nothing further down the pipeline could detect.
  if (result < static_cast<int>(k_gazeStatusSize)) {
    static uint32_t s_shortTransferCount = 0;
    ++s_shortTransferCount;
    if (s_shortTransferCount == 1 || (s_shortTransferCount % 100) == 0) {
      Util::DriverLog("[Gaze] Short transfer: {} of {} bytes, discarding frame (occurrence {}).", result, k_gazeStatusSize, s_shortTransferCount);
    }
    return 0;
  }

  if (state.magic[0] == GAZE_MAGIC_0 && state.magic[1] == GAZE_MAGIC_1_STATE) {
    HmdDeviceHooks::UpdateGaze(&state, sizeof(hmd2_gaze_status_t));
    CustomShareManager *pShareManager = CustomShareManager::getSingleton();
    pShareManager->setGazeStatus(&state);
  }

  return 0;
}
