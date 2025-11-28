#include "usb_thread_hooks.h"

#include "hmd_driver_loader.h"
#include "hook_lib.h"
#include "vr_settings.h"

namespace psvr2_toolkit {

#pragma pack(push, 1)

  struct report_gaze_user_calibration_payload {
    float x;
    float y;
    float z;
    uint8_t value;
  };

#pragma pack(pop)

  void* imuStatusUsbThread = nullptr;

  int (*CaesarUsbThread__report)(void *thisptr, uint8_t reportType, uint16_t reportId, void *payload, uint16_t payloadLength, uint16_t value, uint16_t index, uint16_t command);

  int (*CaesarUsbThreadImuStatus__poll)(void *) = nullptr;
  int CaesarUsbThreadImuStatus__pollHook(void *thisptr) {
    imuStatusUsbThread = thisptr;
    int result = CaesarUsbThreadImuStatus__poll(thisptr);
    gazeStreamSetEnabled(true); // Keep gaze stream enabled
    return result;
  }

  void UsbThreadHooks::InstallHooks() {
    static HmdDriverLoader *pHmdDriverLoader = HmdDriverLoader::Instance();

    CaesarUsbThread__report = decltype(CaesarUsbThread__report)(pHmdDriverLoader->GetBaseAddress() + 0x1283F0);

    if (!VRSettings::GetBool(STEAMVR_SETTINGS_DISABLE_GAZE, SETTING_DISABLE_GAZE_DEFAULT_VALUE)) {
      // CaesarUsbThreadImuStatus::poll
      HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x1268D0),
                           reinterpret_cast<void *>(CaesarUsbThreadImuStatus__pollHook),
                           reinterpret_cast<void **>(&CaesarUsbThreadImuStatus__poll));
    }
  }

  void gazeStreamGetStatus() {}

  void gazeStreamSetEnabled(bool value) {
    if (imuStatusUsbThread) {
      CaesarUsbThread__report(imuStatusUsbThread, true, 12, nullptr, 0, 0, 0, value ? 1 : 2);
    }
  }

  // TODO
  void gazeUserCalibrationGetStatus() {}

  void gazeUserCalibrationStart() {
    if (imuStatusUsbThread) {
      report_gaze_user_calibration_payload payload = {};
      CaesarUsbThread__report(imuStatusUsbThread, true, 13, &payload, sizeof(payload), 0, 0, 1);
    }
  }

  void gazeUserCalibrationCollect(float x, float y, float z) {
    if (imuStatusUsbThread) {
      report_gaze_user_calibration_payload payload = {};
      payload.x = x;
      payload.y = y;
      payload.z = z;
      CaesarUsbThread__report(imuStatusUsbThread, true, 13, &payload, sizeof(payload), 0, 0, 2);
    }
  }

  void gazeUserCalibrationDiscard(float x, float y, float z) {
    if (imuStatusUsbThread) {
      report_gaze_user_calibration_payload payload = {};
      payload.x = x;
      payload.y = y;
      payload.z = z;
      CaesarUsbThread__report(imuStatusUsbThread, true, 13, &payload, sizeof(payload), 0, 0, 3);
    }
  }

  void gazeUserCalibrationComputeAndApply() {
    if (imuStatusUsbThread) {
      report_gaze_user_calibration_payload payload = {};
      CaesarUsbThread__report(imuStatusUsbThread, true, 13, &payload, sizeof(payload), 0, 0, 4);
    }
  }

  // TODO
  void gazeUserCalibrationGetData() {}

  void gazeUserCalibrationStop() {
    if (imuStatusUsbThread) {
      report_gaze_user_calibration_payload payload = {};
      CaesarUsbThread__report(imuStatusUsbThread, true, 13, &payload, sizeof(payload), 0, 0, 6);
    }
  }

  void gazeUserCalibrationSetEnabledEye(uint8_t value) {
    if (imuStatusUsbThread) {
      report_gaze_user_calibration_payload payload = {};
      payload.value = value;
      CaesarUsbThread__report(imuStatusUsbThread, true, 13, &payload, sizeof(payload), 0, 0, 7);
    }
  }

  void gazeUserCalibrationSetPoint(float x, float y, float z) {
    // from reversed sceHmd2 function.
    gazeUserCalibrationCollect(x * 1000.0f, y * 1000.0f, z * 1000.0f);
  }

} // psvr2_toolkit
