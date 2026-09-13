#include "usb_thread_hooks.h"

#include "../driver_interface/caesar_manager.h"
#include "hmd2_gaze.h"
#include "hmd2_gen_data.h"
#include "hmd_device_camera.h"
#include "hmd_driver_loader.h"
#include "utils/hook_lib.h"
#include "utils/driver_settings.h"
#include "util.h"

#include "custom_share_manager.h"
#include <openvr_driver.h>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <system_error>
#include <vector>

namespace psvr2_toolkit {
struct image_data {
  unsigned char magic[2];
  uint16_t version;
  uint32_t total_size;
  uint32_t timestamp;
  uint8_t unk0[4];
  uint16_t image_type;
  uint8_t unk1[22];
  uint32_t custom_data_size;
  uint8_t unk2[20];
  uint8_t unk3[192];
  uint8_t data[2097152];
};

// TODO: only partial
struct CaesarUsbThreadImage {
  unsigned char unk0[0x220];
  image_data image_data;
};

// Resolves the SteamVR user config directory and appends the calibration blob filename.
static bool GetGazeCalibBlobPath(std::filesystem::path &outPath) {
  vr::ETrackedPropertyError err;
  vr::PropertyContainerHandle_t container = vr::VRDriverHandle();
  uint32_t propSize = vr::VRProperties()->GetStringProperty(container, vr::Prop_UserConfigPath_String, nullptr, 0, &err);

  if (propSize == 0) {
    return false;
  }

  std::string configPath(propSize - 1, '\0');
  vr::VRProperties()->GetStringProperty(container, vr::Prop_UserConfigPath_String, configPath.data(), propSize, &err);

  if (err != vr::TrackedProp_Success) {
    return false;
  }

  outPath = std::filesystem::path(configPath) / std::string(HMD2_GAZE_CALIB_BLOB_FILENAME);
  return true;
}

// True when the file already holds exactly these bytes.
static bool GazeCalibBlobMatchesDisk(const std::filesystem::path &filePath, const char *data, uint32_t size) {
  std::error_code ec;
  const auto existingSize = std::filesystem::file_size(filePath, ec);
  if (ec || existingSize != size) {
    return false;
  }

  std::ifstream inFile(filePath, std::ios::binary);
  if (!inFile.is_open()) {
    return false;
  }

  std::vector<char> existing(size);
  if (!inFile.read(existing.data(), size)) {
    return false;
  }

  return std::memcmp(existing.data(), data, size) == 0;
}

// Persists the gaze calibration blob the headset hands us so it can be replayed on the next connect.
// The block walking and bounds checking live in Hmd2ParseGenData (common/hmd2_gen_data.h) so that they
// can be tested without a headset; everything specific to this driver stays here.
int customHandleData(char *buffer, uint32_t bufferSize) {
  return Hmd2ParseGenData(buffer, bufferSize, [](char *data, uint32_t size) {
    // Avoid saving any blank or invalid calibration data.
    if (!Hmd2IsGazeCalibBlobValid(data, size)) {
      return;
    }

    std::filesystem::path filePath;
    if (!GetGazeCalibBlobPath(filePath)) {
      return;
    }

    // The headset re-sends this blob on every connect, so skip the write unless it actually changed.
    // That removes essentially all of the I/O this hook would otherwise do on the USB data thread.
    if (GazeCalibBlobMatchesDisk(filePath, data, size)) {
      return;
    }

    // Write to a temporary and rename over the target. Truncating the real file in place means a crash
    // or power loss between truncate and write leaves the user with no usable calibration.
    std::filesystem::path tempPath = filePath;
    tempPath += ".tmp";

    {
      std::ofstream outFile(tempPath, std::ios::binary | std::ios::trunc);
      if (!outFile.is_open()) {
        Util::DriverLog("[Gaze] Could not open {} for writing.", tempPath.string());
        return;
      }

      outFile.write(data, size);
      outFile.flush();
      if (!outFile.good()) {
        Util::DriverLog("[Gaze] Failed writing calibration blob to {}.", tempPath.string());
        return;
      }
    }

    std::error_code ec;
    std::filesystem::rename(tempPath, filePath, ec);
    if (ec) {
      Util::DriverLog("[Gaze] Could not replace {}: {}", filePath.string(), ec.message());
      std::filesystem::remove(tempPath, ec);
      return;
    }

    Util::DriverLog("[Gaze] Saved calibration blob ({} bytes).", size);
  });
}

int (*CaesarUsbThreadGenData__handleData)(void *, char *, uint32_t) = nullptr;
int CaesarUsbThreadGenData__handleDataHook(void *thisptr, char *buffer, uint32_t bufferSize) {
  int result = CaesarUsbThreadGenData__handleData(thisptr, buffer, bufferSize);
  customHandleData(buffer, bufferSize);
  return result;
}

static bool IsGazeImageStreamEnabled() {
  // Function-local so the lookup happens on the first frame rather than at static init, by which point
  // VRSettings is guaranteed to be up.
  static const bool enabled = DriverSettings::GetBool(STEAMVR_SETTINGS_GAZE_IMAGE_STREAM_ENABLED, SETTING_GAZE_IMAGE_STREAM_ENABLED_DEFAULT_VALUE);
  return enabled;
}

// How much of the fixed-size image slot is actually worth copying. This runs on the USB thread while
// holding the share mutex, so copying 2 MB when the frame is smaller is not free.
//
// The precise meaning of total_size has NOT been confirmed against hardware yet, so anything outside a
// sane range falls back to copying the whole slot -- a wasted copy is recoverable, a truncated IR frame
// silently corrupts the eyelid work in phase 4.1. The first value observed is logged so that the
// assumption can be checked on-device.
static uint32_t GetGazeImageCopySize(const image_data &img) {
  constexpr uint32_t k_headerSize = 0x100;

  static bool s_logged = false;
  if (!s_logged) {
    s_logged = true;
    const uint32_t observedTotalSize = img.total_size;
    const uint32_t observedCustomDataSize = img.custom_data_size;
    Util::DriverLog("[Gaze] First gaze image: total_size={}, custom_data_size={}, slot={} bytes.", observedTotalSize, observedCustomDataSize,
                    k_gazeImageSlotSize);
  }

  if (img.total_size <= k_headerSize || img.total_size > k_gazeImageSlotSize) {
    return k_gazeImageSlotSize;
  }

  return img.total_size;
}

int (*CaesarUsbThreadImage__poll)(void *thisptr) = nullptr;
int CaesarUsbThreadImage__pollHook(void *thisptr) {
  int result = CaesarUsbThreadImage__poll(thisptr);

  if (result == 0) {
    CaesarUsbThreadImage *a1 = (CaesarUsbThreadImage *)thisptr;
    if (a1->image_data.magic[0] == 'V' && a1->image_data.magic[1] == 'I') {
      if (a1->image_data.image_type == 6) {
        if (IsGazeImageStreamEnabled()) {
          CustomShareManager::getSingleton()->setGazeImage((unsigned char *)&a1->image_data, GetGazeImageCopySize(a1->image_data));
        }
      } else if (a1->image_data.image_type == 11) {
        static HmdDeviceCamera *pHmdDeviceCamera = HmdDeviceCamera::Instance();

        int64_t hmdToHostOffset;
        CaesarManager::getSingleton()->getIMUTimestampOffset(&hmdToHostOffset);

        double timeOffset = (static_cast<int64_t>(a1->image_data.timestamp) + hmdToHostOffset) / 1e6;

        static LARGE_INTEGER frequency{};
        if (frequency.QuadPart == 0) {
          QueryPerformanceFrequency(&frequency);
        }

        uint64_t ticks = static_cast<uint64_t>(timeOffset * static_cast<double>(frequency.QuadPart));

        pHmdDeviceCamera->UploadBC4(ticks, a1->image_data.data);
      }
    }
  }

  return result;
}

void UsbThreadHooks::InstallHooks() {
  static HmdDriverLoader *pHmdDriverLoader = HmdDriverLoader::Instance();

  // CaesarUsbThreadImage::poll
  HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x125E00), reinterpret_cast<void *>(CaesarUsbThreadImage__pollHook),
                       reinterpret_cast<void **>(&CaesarUsbThreadImage__poll));

  // CaesarUsbThreadGenData::handleData
  HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x127250),
                       reinterpret_cast<void *>(CaesarUsbThreadGenData__handleDataHook), reinterpret_cast<void **>(&CaesarUsbThreadGenData__handleData));
}

} // namespace psvr2_toolkit
