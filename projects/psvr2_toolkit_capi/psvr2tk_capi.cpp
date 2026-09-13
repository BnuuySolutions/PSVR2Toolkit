#include "psvr2tk_capi.h"
#include "common.h"
#include "psvr2tk_capi_private.h"
#include "custom_share_manager.h"
#include "pupillometry.h"
#include "util.h"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>

#ifdef _WIN32
#include <windows.h>
#endif

extern "C" {
static int g_slot = -1;

// Per-thread, not global: these track "last frame this caller saw". Sharing them across threads makes
// two pollers steal each other's new-frame edges, so each would silently miss roughly half the stream.
static thread_local int g_lastGazeStatusCounter = -1;
static thread_local int g_lastGazeImageCounter = -1;

int psvr2_toolkit_init() {
  CustomShareManager::createSingleton();

  if (g_slot < 0) {
    g_slot = CustomShareManager::getSingleton()->claimSlot();
  }
  return g_slot >= 0 ? PSVR2TK_RESULT_OK : PSVR2TK_RESULT_NO_SLOT;
}

void psvr2_toolkit_deinit() {
  if (g_slot >= 0) {
    CustomShareManager::getSingleton()->releaseSlot(g_slot);
    g_slot = -1;
  }
}

bool psvr2_toolkit_get_driver_active() { return CustomShareManager::getSingleton()->getDriverActive(); }

// Note: unlike the command-submitting entry points below, the streaming reads deliberately do not call
// getDriverActive(). That check costs two named-mutex operations, and on a path polled at the gaze
// frame rate it buys nothing the blocking timeout does not already report.
bool psvr2_toolkit_gaze_status(hmd2_gaze_status_t *pGazeStatus, uint32_t timeoutMs) {
  if (!pGazeStatus) {
    return false;
  }

  return CustomShareManager::getSingleton()->getGazeStatus(pGazeStatus, &g_lastGazeStatusCounter, timeoutMs);
}

// Deprecated: hands back a pointer into shared memory that the producer may recycle while the caller
// is still reading it. Prefer psvr2_toolkit_gaze_image_copy, which copies under the lock.
bool psvr2_toolkit_gaze_image(unsigned char **pGazeImage, uint32_t timeoutMs) {
  if (!pGazeImage) {
    return false;
  }

  return CustomShareManager::getSingleton()->getGazeImageBuffer(pGazeImage, &g_lastGazeImageCounter, timeoutMs);
}

bool psvr2_toolkit_gaze_image_copy(unsigned char *pDest, uint32_t destSize, uint32_t *pOutSize, uint32_t timeoutMs) {
  if (!pDest || destSize == 0) {
    return false;
  }

  return CustomShareManager::getSingleton()->getGazeImageCopy(pDest, destSize, pOutSize, &g_lastGazeImageCounter, timeoutMs);
}

int psvr2_toolkit_write_pcm(VRControllerType controllerType, const unsigned char *pcm) {
  if (!CustomShareManager::getSingleton()->getDriverActive()) {
    return PSVR2TK_RESULT_DRIVER_INACTIVE;
  }

  if (controllerType > VRControllerType::Both) {
    return PSVR2TK_RESULT_INVALID_PARAMETER;
  }

  if (g_slot >= 0) {
    CustomShareManager::getSingleton()->writePcm(g_slot, controllerType, pcm);
  } else {
    return PSVR2TK_RESULT_NO_SLOT;
  }

  return PSVR2TK_RESULT_OK;
}

int psvr2_toolkit_wait_for_pcm() {
  if (!CustomShareManager::getSingleton()->getDriverActive()) {
    return PSVR2TK_RESULT_DRIVER_INACTIVE;
  }

  bool success = CustomShareManager::getSingleton()->waitForPcmUpdate();
  return success ? PSVR2TK_RESULT_OK : PSVR2TK_RESULT_TIMEOUT;
}

int psvr2_toolkit_set_trigger_effect(VRControllerType controllerType, const ScePadTriggerEffectCommand &command) {
  if (!CustomShareManager::getSingleton()->getDriverActive()) {
    return PSVR2TK_RESULT_DRIVER_INACTIVE;
  }

  if (controllerType > VRControllerType::Both) {
    return PSVR2TK_RESULT_INVALID_PARAMETER;
  }

  if (g_slot >= 0) {
    DriverCommand drvCmd = {};
    drvCmd.type = DriverCommandType::TriggerEffectSet;
    drvCmd.triggerEffect.slot = g_slot;
    drvCmd.triggerEffect.payload.controllerType = controllerType;
    drvCmd.triggerEffect.payload.command = command;
    bool success = CustomShareManager::getSingleton()->submitCommand(drvCmd);

    return success ? PSVR2TK_RESULT_OK : PSVR2TK_RESULT_TIMEOUT;
  } else {
    return PSVR2TK_RESULT_NO_SLOT;
  }
}

// --- Pupillometry ------------------------------------------------------------------------------
//
// The processing runs HERE, in the client's own process, rather than in the driver. That keeps the
// driver, the shared-memory layout and the IPC protocol untouched, and it is also more correct: the
// baseline and activity windows are per-consumer state, and two applications watching the same headset
// legitimately want their own.
//
// One processor per thread, because the pipeline is a continuous filter over the sample stream and two
// callers sharing one would tear each other's history apart. Each thread also keeps its own "last frame
// seen" counter, independent of the one psvr2_toolkit_gaze_status uses, so an application can poll both
// without either stealing the other's frames.

static std::mutex g_pupilAxesMutex;
static bool g_pupilAxesLoaded = false;
static float g_pupilAxes[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // leftAz, leftEl, rightAz, rightEl

// Camera axes live next to psvr2_toolkit_capi.dll, so a calibration performed in one application is
// picked up by every other one on the machine.
static bool GetPupilAxesPath(std::filesystem::path &outPath) {
#ifdef _WIN32
  HMODULE hModule = NULL;
  if (!GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, (LPCSTR)&GetPupilAxesPath, &hModule)) {
    return false;
  }

  char path[MAX_PATH] = {};
  if (GetModuleFileNameA(hModule, path, MAX_PATH) == 0) {
    return false;
  }

  outPath = std::filesystem::path(path).parent_path() / "pupil_camera_axes.txt";
  return true;
#else
  (void)outPath;
  return false;
#endif
}

static void LoadPupilAxesOnce() {
  std::lock_guard<std::mutex> lock(g_pupilAxesMutex);
  if (g_pupilAxesLoaded) {
    return;
  }
  g_pupilAxesLoaded = true;

  std::filesystem::path path;
  if (!GetPupilAxesPath(path)) {
    return;
  }

  std::ifstream in(path);
  if (!in.is_open()) {
    return; // uncalibrated; the straight-ahead default still runs, just less accurately
  }

  float values[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  if (in >> values[0] >> values[1] >> values[2] >> values[3]) {
    for (int i = 0; i < 4; ++i) {
      g_pupilAxes[i] = values[i];
    }
  }
}

static PupillometryProcessor &ThreadPupillometry() {
  static thread_local PupillometryProcessor processor;
  static thread_local bool configured = false;
  static thread_local float appliedAxes[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  LoadPupilAxesOnce();

  float axes[4];
  {
    std::lock_guard<std::mutex> lock(g_pupilAxesMutex);
    for (int i = 0; i < 4; ++i) {
      axes[i] = g_pupilAxes[i];
    }
  }

  // Reconfigure only when the axes actually change, since that resets the filter history.
  bool changed = !configured;
  for (int i = 0; i < 4 && !changed; ++i) {
    changed = appliedAxes[i] != axes[i];
  }

  if (changed) {
    PupillometryConfig config;
    config.leftCameraAzimuthDeg = axes[0];
    config.leftCameraElevationDeg = axes[1];
    config.rightCameraAzimuthDeg = axes[2];
    config.rightCameraElevationDeg = axes[3];
    processor.Configure(config);
    processor.Reset();

    for (int i = 0; i < 4; ++i) {
      appliedAxes[i] = axes[i];
    }
    configured = true;
  }

  return processor;
}

static PupilEyeSample ToPupilEyeSample(const hmd2_gaze_wearable_eye_t &eye) {
  PupilEyeSample out;
  out.diameterMm = eye.pupil_dia_mm;
  out.diameterValid = eye.is_pupil_dia_valid == HMD2_GAZE_BOOL_TRUE;
  out.blinking = eye.is_blink_valid == HMD2_GAZE_BOOL_TRUE && eye.blink == HMD2_GAZE_BOOL_TRUE;
  // Headset-space direction, +Z forward, exactly as the device reports it.
  out.gazeDirX = eye.gaze_dir_norm.x;
  out.gazeDirY = eye.gaze_dir_norm.y;
  out.gazeDirZ = eye.gaze_dir_norm.z;
  out.gazeValid = eye.is_gaze_dir_valid == HMD2_GAZE_BOOL_TRUE;
  return out;
}

int psvr2_toolkit_pupillometry(Psvr2tkPupillometry *pOut, uint32_t timeoutMs) {
  if (!pOut) {
    return PSVR2TK_RESULT_INVALID_PARAMETER;
  }

  static thread_local int lastCounter = -1;

  hmd2_gaze_status_t status = {};
  if (!CustomShareManager::getSingleton()->getGazeStatus(&status, &lastCounter, timeoutMs)) {
    return PSVR2TK_RESULT_TIMEOUT;
  }

  PupilFrame frame;
  frame.timestampUs = status.wearable.timestamp;
  frame.left = ToPupilEyeSample(status.wearable.left);
  frame.right = ToPupilEyeSample(status.wearable.right);

  PupillometryResult result;
  if (!ThreadPupillometry().Update(frame, result)) {
    // Still inside the blink guard band; no verdict on this frame yet.
    return PSVR2TK_RESULT_TIMEOUT;
  }

  *pOut = {};
  pOut->timestamp = result.timestampUs;
  pOut->isValid = result.valid ? 1u : 0u;
  pOut->isBaselineReady = result.baselineReady ? 1u : 0u;
  pOut->isActivityReady = result.activityReady ? 1u : 0u;
  pOut->contributingEyes = static_cast<uint32_t>(result.contributingEyes);
  pOut->correctedMm = result.correctedMm;
  pOut->smoothedMm = result.smoothedMm;
  pOut->baselineMm = result.baselineMm;
  pOut->deltaMm = result.deltaMm;
  pOut->relative = result.relative;
  pOut->activityIndex = result.activityIndex;

  return PSVR2TK_RESULT_OK;
}

int psvr2_toolkit_pupillometry_set_camera_axes(float leftAzimuthDeg, float leftElevationDeg, float rightAzimuthDeg, float rightElevationDeg) {
  // A camera more than 60 degrees off the eye's forward axis is not a plausible fit result.
  const float candidates[4] = {leftAzimuthDeg, leftElevationDeg, rightAzimuthDeg, rightElevationDeg};
  for (float v : candidates) {
    if (!(v > -60.0f && v < 60.0f)) {
      return PSVR2TK_RESULT_INVALID_PARAMETER;
    }
  }

  {
    std::lock_guard<std::mutex> lock(g_pupilAxesMutex);
    g_pupilAxesLoaded = true; // an explicit set supersedes whatever is on disk
    for (int i = 0; i < 4; ++i) {
      g_pupilAxes[i] = candidates[i];
    }
  }

  // Persist so other applications pick the calibration up. Failing to write is not fatal: the axes are
  // already live for this process.
  std::filesystem::path path;
  if (GetPupilAxesPath(path)) {
    std::ofstream out(path, std::ios::trunc);
    if (out.is_open()) {
      out << candidates[0] << " " << candidates[1] << " " << candidates[2] << " " << candidates[3] << "\n";
    }
  }

  return PSVR2TK_RESULT_OK;
}

int psvr2_toolkit_pupillometry_get_camera_axes(float *pLeftAzimuthDeg, float *pLeftElevationDeg, float *pRightAzimuthDeg, float *pRightElevationDeg) {
  if (!pLeftAzimuthDeg || !pLeftElevationDeg || !pRightAzimuthDeg || !pRightElevationDeg) {
    return PSVR2TK_RESULT_INVALID_PARAMETER;
  }

  LoadPupilAxesOnce();

  std::lock_guard<std::mutex> lock(g_pupilAxesMutex);
  *pLeftAzimuthDeg = g_pupilAxes[0];
  *pLeftElevationDeg = g_pupilAxes[1];
  *pRightAzimuthDeg = g_pupilAxes[2];
  *pRightElevationDeg = g_pupilAxes[3];
  return PSVR2TK_RESULT_OK;
}

int psvr2_toolkit_set_hmd_rumble(uint8_t rumbleHz) {
  if (!CustomShareManager::getSingleton()->getDriverActive()) {
    return PSVR2TK_RESULT_DRIVER_INACTIVE;
  }

  // Headset does not do more than 25 hz.
  // The range is technically 10-25 hz, but the headset does accept numbers in the 1-9 range and makes it 10.
  // And we also want to support 0 hz to allow stopping the motor.
  if (rumbleHz > 25) {
    return PSVR2TK_RESULT_INVALID_PARAMETER;
  }

  DriverCommand drvCmd = {};
  drvCmd.type = DriverCommandType::HeadsetRumbleSet;
  drvCmd.headsetRumble.rumbleHz = rumbleHz;
  bool success = CustomShareManager::getSingleton()->submitCommand(drvCmd);

  return success ? PSVR2TK_RESULT_OK : PSVR2TK_RESULT_TIMEOUT;
}

static int SendGazeCalibrationCommand(DriverCommandType type, GazeCalibrationCommand *pCommand) {
  if (!pCommand) {
    return PSVR2TK_RESULT_INVALID_PARAMETER;
  }

  if (!CustomShareManager::getSingleton()->getDriverActive()) {
    return PSVR2TK_RESULT_DRIVER_INACTIVE;
  }

  DriverCommand drvCmd = {};
  drvCmd.type = type;
  drvCmd.gazeCalibration = *pCommand;

  const bool fulfilled = CustomShareManager::getSingleton()->submitCommand(drvCmd);
  *pCommand = drvCmd.gazeCalibration;

  return fulfilled ? PSVR2TK_RESULT_OK : PSVR2TK_RESULT_TIMEOUT;
}

// Deprecated: discards submitCommand's result, so a five-second timeout is indistinguishable from a
// successful calibration step. Prefer the _checked variants below.
GazeCalibrationCommand psvr2_toolkit_private_send_gaze_set_command(GazeCalibrationCommand command) {
  SendGazeCalibrationCommand(DriverCommandType::GazeCalibrationSet, &command);
  return command;
}

GazeCalibrationCommand psvr2_toolkit_private_send_gaze_get_command(GazeCalibrationCommand command) {
  SendGazeCalibrationCommand(DriverCommandType::GazeCalibrationGet, &command);
  return command;
}

int psvr2_toolkit_private_send_gaze_set_command_checked(GazeCalibrationCommand *pCommand) {
  return SendGazeCalibrationCommand(DriverCommandType::GazeCalibrationSet, pCommand);
}

int psvr2_toolkit_private_send_gaze_get_command_checked(GazeCalibrationCommand *pCommand) {
  return SendGazeCalibrationCommand(DriverCommandType::GazeCalibrationGet, pCommand);
}

int psvr2_toolkit_private_set_usb_connection_state(bool connected) {
  if (!CustomShareManager::getSingleton()->getDriverActive()) {
    return PSVR2TK_RESULT_DRIVER_INACTIVE;
  }

  DriverCommand drvCmd = {};
  drvCmd.type = DriverCommandType::UsbConnectionStateSet;
  drvCmd.usbConnection.isConnected = connected;
  bool success = CustomShareManager::getSingleton()->submitCommand(drvCmd);

  return success ? PSVR2TK_RESULT_OK : PSVR2TK_RESULT_TIMEOUT;
}
}