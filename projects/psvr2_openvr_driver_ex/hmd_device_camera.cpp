#include "hmd_device_camera.h"
#include "hmd_math.h"
#include "img_utils.h"
#include "util.h"

#include "driver_interface/caesar_manager.h"
#include "driver_interface/share_manager.h"

#include <openvr_driver.h>
#include <cmath>
#include <utility>
#include <string>

psvr2_toolkit::HmdDeviceCamera *g_pHmdDeviceCamera;

namespace psvr2_toolkit {

const float k_ZoomFactor = 3.25f;

static bool solve4x4(double M[4][4], double V[4], double k[4]) {
  for (int i = 0; i < 4; ++i) {
    int pivot = i;
    for (int j = i + 1; j < 4; ++j) {
      if (std::abs(M[j][i]) > std::abs(M[pivot][i])) {
        pivot = j;
      }
    }
    if (pivot != i) {
      for (int col = 0; col < 4; ++col) {
        std::swap(M[i][col], M[pivot][col]);
      }
      std::swap(V[i], V[pivot]);
    }
    if (std::abs(M[i][i]) < 1e-12) {
      return false;
    }
    for (int row = i + 1; row < 4; ++row) {
      double factor = M[row][i] / M[i][i];
      for (int col = i; col < 4; ++col) {
        M[row][col] -= factor * M[i][col];
      }
      V[row] -= factor * V[i];
    }
  }
  for (int i = 3; i >= 0; --i) {
    double sum = 0.0;
    for (int col = i + 1; col < 4; ++col) {
      sum += M[i][col] * k[col];
    }
    k[i] = (V[i] - sum) / M[i][i];
  }
  return true;
}

#pragma pack(push, 1)
struct CameraConfigData {
  uint32_t camId;
  uint16_t widthPx;
  uint16_t heightPx;
  float pxMat[9];
  double coff[20];
  uint32_t zeros[6];
};
#pragma pack(pop)

bool HmdDeviceCamera::LoadCalibrationFromShareManager() {
  ShareManager *pShareManager = ShareManager::GetInstance();
  if (!pShareManager) {
    return false;
  }

  uint8_t calibBuffer[0x800] = {0};
  uint32_t counter = 0;

  // Wait until we have calibration data
  while (counter == 0) {
    pShareManager->ReadCalib_0x50c(calibBuffer, &counter);
    _mm_pause();
  }

  const CameraConfigData *configs = reinterpret_cast<const CameraConfigData *>(calibBuffer + 0x18);

  bool anyLoaded = false;
  for (int i = 0; i < 4; ++i) {
    uint32_t camId = configs[i].camId;
    if (camId < 2 && configs[i].pxMat[0] > 0.0f) {
      m_calibration[camId].fx = configs[i].pxMat[0];
      m_calibration[camId].fy = configs[i].pxMat[4];
      m_calibration[camId].cx = configs[i].pxMat[2];
      m_calibration[camId].cy = configs[i].pxMat[5];
      std::memcpy(m_calibration[camId].params.params, configs[i].coff, sizeof(double) * 20);
      m_calibration[camId].loaded = true;
      anyLoaded = true;

      Util::DriverLog("Loaded calibration for camera {} from ShareManager: fx={}, fy={}, cx={}, cy={}", camId, m_calibration[camId].fx, m_calibration[camId].fy,
                      m_calibration[camId].cx, m_calibration[camId].cy);
    }
  }
  return anyLoaded;
}

void HmdDeviceCamera::FitDistortionCoefficients() {
  for (uint32_t cameraIndex = 0; cameraIndex < 2; ++cameraIndex) {
    float fx = m_calibration[cameraIndex].fx;
    float fy = m_calibration[cameraIndex].fy;
    float cx = m_calibration[cameraIndex].cx;
    float cy = m_calibration[cameraIndex].cy;
    const DistortionParameters &params = m_calibration[cameraIndex].params;

    alignas(16) double S[19] = {0};
    alignas(16) double V[4] = {0};

    const int numSamples = 20;
    for (int iu = 0; iu <= numSamples; ++iu) {
      float flInputU = (float)iu / numSamples;
      for (int iv = 0; iv <= numSamples; ++iv) {
        float flInputV = (float)iv / numSamples;

        // TODO: check if cx needs to be flipped here based on camera index
        float x = (flInputU * IMAGE_WIDTH - cx) / fx;
        float y = (flInputV * IMAGE_HEIGHT - cy) / fy;

        float r = std::sqrt(x * x + y * y);
        if (r < 1e-5f)
          continue;

        float out[2];
        applyDistortionTransform(out, &params, x, y);

        float u_d = out[0] * fx + cx;
        float v_d = out[1] * fy + cy;
        float dist_from_center_sq = (u_d - cx) * (u_d - cx) + (v_d - cy) * (v_d - cy);
        if (dist_from_center_sq > 508.f * 508.f) {
          continue;
        }

        double theta = std::atan(r);
        double r_d = std::sqrt(out[0] * out[0] + out[1] * out[1]);

        double theta_p[19];
        theta_p[0] = 1.0;
        for (int p = 1; p <= 18; ++p) {
          theta_p[p] = theta_p[p - 1] * theta;
        }

        for (int p = 6; p <= 18; p += 2) {
          S[p] += theta_p[p];
        }

        V[0] += theta_p[3] * (r_d - theta);
        V[1] += theta_p[5] * (r_d - theta);
        V[2] += theta_p[7] * (r_d - theta);
        V[3] += theta_p[9] * (r_d - theta);
      }
    }

    alignas(16) double M[4][4] = {{S[6], S[8], S[10], S[12]}, {S[8], S[10], S[12], S[14]}, {S[10], S[12], S[14], S[16]}, {S[12], S[14], S[16], S[18]}};

    alignas(16) double k[4] = {0};
    if (solve4x4(M, V, k)) {
      fittedCoefficients[cameraIndex][0] = k[0];
      fittedCoefficients[cameraIndex][1] = k[1];
      fittedCoefficients[cameraIndex][2] = k[2];
      fittedCoefficients[cameraIndex][3] = k[3];
      fittedCoefficients[cameraIndex][4] = 0.0;
      fittedCoefficients[cameraIndex][5] = 0.0;
      fittedCoefficients[cameraIndex][6] = 0.0;
      fittedCoefficients[cameraIndex][7] = 0.0;

      vr::VRDriverLog()->Log(("Fitted camera " + std::to_string(cameraIndex) + " coefficients: " + std::to_string(k[0]) + ", " + std::to_string(k[1]) + ", " +
                              std::to_string(k[2]) + ", " + std::to_string(k[3]))
                                 .c_str());
    } else {
      vr::VRDriverLog()->Log(("Failed to solve least squares for camera " + std::to_string(cameraIndex)).c_str());
    }
  }
}

HmdDeviceCamera::HmdDeviceCamera() {
  for (uint32_t i = 0; i < 2; ++i) {
    m_calibration[i].params = DistortionParameters{{0}};
  }
}

HmdDeviceCamera *HmdDeviceCamera::Instance() {
  if (!g_pHmdDeviceCamera) {
    g_pHmdDeviceCamera = new HmdDeviceCamera;
  }

  return g_pHmdDeviceCamera;
}

void HmdDeviceCamera::LoadCalibration() {
  g_pHmdDeviceCamera->LoadCalibrationFromShareManager();
  g_pHmdDeviceCamera->FitDistortionCoefficients();
}

void HmdDeviceCamera::UploadBC4(uint64_t tickTime, uint8_t *data) {
  if (pVRBlockQueue != nullptr && shouldSubmit) {
    vr::PropertyContainerHandle_t handle;
    void *blockImageBuffer;

    auto r = pVRBlockQueue->AcquireWriteOnlyBlock(blockQueueHandle, &handle, &blockImageBuffer);
    if (r == vr::EBlockQueueError::BlockQueueError_None) {
      vr::EVRInitError eError;
      static vr::IVRPaths *pVRPaths = (vr::IVRPaths *)vr::VRDriverContext()->GetGenericInterface(vr::IVRPaths_Version, &eError);

      WritePathProperty(pVRPaths, handle, "/server_time_ticks", tickTime);

      WritePathProperty(pVRPaths, handle, "/frame_sequence", (uint64_t)frameSequence++);
      WritePathProperty(pVRPaths, handle, "/frame_size", (int32_t)frameDataSize);

      double frameTime = frameSequence * (1.0 / 59.94);
      WritePathProperty(pVRPaths, handle, "/frame_time_monotonic", frameTime);
      WritePathProperty(pVRPaths, handle, "/delivery_rate", 59.94);
      WritePathProperty(pVRPaths, handle, "/elapsed_time", 1.0 / 59.94);

      // Convert and Release
      if (!Util::IsRunningOnWine()) {
        static BC4_to_NV12_Converter conv(TEXTURE_WIDTH, IMAGE_WIDTH, IMAGE_HEIGHT);
        conv.convert(data, data + BC4_DATA_SIZE, reinterpret_cast<uint8_t *>(blockImageBuffer));
      } else {
        // GPU-based decoding isn't supported under Wine/Proton.
        static BC4_to_NV12_Converter_CPU conv(TEXTURE_WIDTH, IMAGE_WIDTH, IMAGE_HEIGHT);
        conv.convert(data, data + BC4_DATA_SIZE, reinterpret_cast<uint8_t *>(blockImageBuffer));
      }

      pVRBlockQueue->ReleaseWriteOnlyBlock(blockQueueHandle, handle);
    }
  }
}

// TODO: I believe false = failure, true = success?

bool HmdDeviceCamera::GetCameraFrameDimensions(vr::ECameraVideoStreamFormat nVideoStreamFormat, uint32_t *pWidth, uint32_t *pHeight) {
  vr::VRDriverLog()->Log(__FUNCTION__);

  *pWidth = IMAGE_WIDTH * 2;
  *pHeight = IMAGE_HEIGHT;

  return true;
}

bool HmdDeviceCamera::GetCameraFrameBufferingRequirements(int *pDefaultFrameQueueSize, uint32_t *pFrameBufferDataSize) { return false; }

bool HmdDeviceCamera::SetCameraFrameBuffering(int nFrameBufferCount, void **ppFrameBuffers, uint32_t nFrameBufferDataSize) { return false; }

bool HmdDeviceCamera::SetCameraVideoStreamFormat(vr::ECameraVideoStreamFormat nVideoStreamFormat) { return false; }

vr::ECameraVideoStreamFormat HmdDeviceCamera::GetCameraVideoStreamFormat() {
  vr::VRDriverLog()->Log(__FUNCTION__);
  return vr::CVS_FORMAT_NV12;
}

bool HmdDeviceCamera::StartVideoStream() {
  vr::VRDriverLog()->Log(__FUNCTION__);
  // TODO: is this correct?
  shouldSubmit = true;
  SetUserBit(CameraUser_Hmd, true);
  return true;
}

void HmdDeviceCamera::StopVideoStream() {
  vr::VRDriverLog()->Log(__FUNCTION__);
  // TODO: is this correct?
  shouldSubmit = false;
  SetUserBit(CameraUser_Hmd, false);
}

bool HmdDeviceCamera::IsVideoStreamActive(bool *pbPaused, float *pflElapsedTime) {
  vr::VRDriverLog()->Log(__FUNCTION__);
  *pbPaused = !shouldSubmit;
  // TODO: track actual elapsed time!
  *pflElapsedTime = shouldSubmit ? 100.0f : 0.0f;
  return true;
}

const vr::CameraVideoStreamFrame_t *HmdDeviceCamera::GetVideoStreamFrame() {
  vr::VRDriverLog()->Log(__FUNCTION__);
  return nullptr;
}

void HmdDeviceCamera::ReleaseVideoStreamFrame(const vr::CameraVideoStreamFrame_t *pFrameImage) {
  // ...
}

bool HmdDeviceCamera::SetAutoExposure(bool bEnable) { return false; }

bool HmdDeviceCamera::PauseVideoStream() {
  vr::VRDriverLog()->Log(__FUNCTION__);
  // TODO: is this correct?
  shouldSubmit = false;
  SetUserBit(CameraUser_Hmd, false);
  return true;
}

bool HmdDeviceCamera::ResumeVideoStream() {
  vr::VRDriverLog()->Log(__FUNCTION__);
  // TODO: is this correct?
  shouldSubmit = true;

  SetUserBit(CameraUser_Hmd, true);

  return true;
}

bool HmdDeviceCamera::GetCameraDistortion(uint32_t nCameraIndex, float flInputU, float flInputV, float *pflOutputU, float *pflOutputV) {
  if (nCameraIndex >= 2)
    return false;

  flInputU *= IMAGE_WIDTH;
  flInputV *= IMAGE_HEIGHT;

  const auto &calib = m_calibration[nCameraIndex];

  // The center seems to be from the right for the right eye. Weird.
  const float cx = nCameraIndex == 0 ? calib.cx : (IMAGE_WIDTH - calib.cx);
  const float cy = calib.cy;

  float out[2];
  applyDistortionTransform(out, &calib.params, ((flInputU - cx) / calib.fx) * k_ZoomFactor, ((flInputV - cy) / calib.fy) * k_ZoomFactor);

  *pflOutputU = ((out[0] * calib.fx) + cx) / IMAGE_WIDTH;
  *pflOutputV = ((out[1] * calib.fy) + cy) / IMAGE_HEIGHT;

  return true;
}

bool HmdDeviceCamera::GetCameraProjection(uint32_t nCameraIndex, vr::EVRTrackedCameraFrameType eFrameType, float flZNear, float flZFar,
                                          vr::HmdMatrix44_t *pProjection) {
  if (nCameraIndex >= 2)
    return false;

  vr::VRDriverLog()->Log(__FUNCTION__);

  Util::DriverLog("camera: {}", nCameraIndex);

  const auto &calib = m_calibration[nCameraIndex];
  const float fx = calib.fx;
  const float fy = calib.fy;

  // The center seems to be from the right for the right eye. Weird.
  const float cx = nCameraIndex == 0 ? calib.cx : (IMAGE_WIDTH - calib.cx);
  const float cy = calib.cy;
  const float image_width = IMAGE_WIDTH * 2.0f;
  const float image_height = IMAGE_HEIGHT;

  // Convert pixel-space intrinsics to view-space projection plane boundaries at z_near
  // This defines the frustum shape that matches the camera sensor
  float left = -cx * (flZNear) / fx;
  float right = (image_width - cx) * (flZNear) / fx;
  float bottom = -(image_height - cy) * (flZNear) / fy;
  float top = cy * (flZNear) / fy;
  *pProjection = HmdMath::createProjectionMatrix(flZNear, flZFar, left * k_ZoomFactor, right * k_ZoomFactor, top * k_ZoomFactor, bottom * k_ZoomFactor);

  return true;
}

bool HmdDeviceCamera::SetFrameRate(int nISPFrameRate, int nSensorFrameRate) { return false; }

bool HmdDeviceCamera::SetCameraVideoSinkCallback(vr::ICameraVideoSinkCallback *pCameraVideoSinkCallback) { return false; }

bool HmdDeviceCamera::GetCameraCompatibilityMode(vr::ECameraCompatibilityMode *pCameraCompatibilityMode) { return false; }

bool HmdDeviceCamera::SetCameraCompatibilityMode(vr::ECameraCompatibilityMode nCameraCompatibilityMode) { return false; }

bool HmdDeviceCamera::GetCameraFrameBounds(vr::EVRTrackedCameraFrameType eFrameType, uint32_t *pLeft, uint32_t *pTop, uint32_t *pWidth, uint32_t *pHeight) {
  vr::VRDriverLog()->Log(__FUNCTION__);

  // TODO: HACK!!
  if (pLeft) {
    *pLeft = 0;
  }
  if (pTop) {
    *pTop = 0;
  }
  if (pWidth) {
    *pWidth = IMAGE_WIDTH * 2;
  }
  if (pHeight) {
    *pHeight = IMAGE_HEIGHT;
  }
  return true;
}

bool HmdDeviceCamera::GetCameraIntrinsics(uint32_t nCameraIndex, vr::EVRTrackedCameraFrameType eFrameType, vr::HmdVector2_t *pFocalLength,
                                          vr::HmdVector2_t *pCenter, vr::EVRDistortionFunctionType *peDistortionType,
                                          double rCoefficients[vr::k_unMaxDistortionFunctionParameters]) {
  if (nCameraIndex >= 2)
    return false;

  const auto &calib = m_calibration[nCameraIndex];
  float fx = calib.fx;
  float fy = calib.fy;
  float cx = calib.cx;
  float cy = calib.cy;

  if (eFrameType == vr::VRTrackedCameraFrameType_Distorted) {
    if (pFocalLength) {
      pFocalLength->v[0] = fx;
      pFocalLength->v[1] = fy;
    }
    if (pCenter) {
      pCenter->v[0] = cx;
      pCenter->v[1] = cy;
    }
    if (peDistortionType)
      *peDistortionType = vr::VRDistortionFunctionType_Extended_FTheta;
    if (rCoefficients)
      std::memcpy(rCoefficients, fittedCoefficients[nCameraIndex], sizeof(double) * 8);
  } else {
    if (pFocalLength) {
      pFocalLength->v[0] = fx / k_ZoomFactor;
      pFocalLength->v[1] = fy / k_ZoomFactor;
    }

    if (pCenter) {
      pCenter->v[0] = cx;
      pCenter->v[1] = cy;
    }

    if (peDistortionType)
      *peDistortionType = vr::VRDistortionFunctionType_None;
    if (rCoefficients)
      std::memcpy(rCoefficients, fittedCoefficients[nCameraIndex], sizeof(double) * 8);
  }

  return true;
}

bool HmdDeviceCamera::CameraStreamEnabled() {
  std::lock_guard<std::mutex> lock(m_cameraUserMutex);

  bool cameraShouldBeOn = (cameraUserBitmask != 0);

  return cameraShouldBeOn;
}

void HmdDeviceCamera::SetUserBit(CameraUser user, bool enable) {
  std::lock_guard<std::mutex> lock(m_cameraUserMutex);

  uint8_t oldBitmask = cameraUserBitmask;
  if (enable) {
    cameraUserBitmask |= static_cast<uint8_t>(user);
  } else {
    cameraUserBitmask &= ~static_cast<uint8_t>(user);
  }

  bool oldCameraShouldBeOn = (oldBitmask != 0);
  bool newCameraShouldBeOn = (cameraUserBitmask != 0);

  if (oldCameraShouldBeOn != newCameraShouldBeOn) {
    Util::DriverLog("Camera state changed to: {}", newCameraShouldBeOn ? "ON" : "OFF");
    char data[8] = {1, 0, 0, 0, (char)(newCameraShouldBeOn ? 0x10 : 0x05), 0, 0, 0};
    auto singleton = CaesarManager::getSingleton();
    if (singleton && singleton->imageThread) {
      singleton->imageThread->ControlCommand(true, 0xb, data, 8, 0, 0, 1);
    }
  }
}

} // namespace psvr2_toolkit
