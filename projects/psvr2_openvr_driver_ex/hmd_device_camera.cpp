#include "hmd_device_camera.h"
#include "hmd_device_hooks.h"
#include "img_utils.h"
#include "util.h"

#include <filesystem>
#include <fstream>

psvr2_toolkit::HmdDeviceCamera *g_pHmdDeviceCamera;

namespace psvr2_toolkit {

  HmdDeviceCamera *HmdDeviceCamera::Instance() {
    if (!g_pHmdDeviceCamera) {
      g_pHmdDeviceCamera = new HmdDeviceCamera;
    }

    return g_pHmdDeviceCamera;
  }
  static bool hasSavedDebugImage = false;
  void HmdDeviceCamera::UploadBC4(uint64_t tickTime, uint8_t* data) {
      if (pVRBlockQueue != nullptr && shouldSubmit)
      {
          vr::PropertyContainerHandle_t handle;
          void* blockImageBuffer;

          auto r = pVRBlockQueue->AcquireWriteOnlyBlock(blockQueueHandle, &handle, &blockImageBuffer);
          if (r == vr::EBlockQueueError::BlockQueueError_None)
          {
              vr::EVRInitError eError;
              static vr::IVRPaths* pVRPaths = (vr::IVRPaths*)vr::VRDriverContext()->GetGenericInterface(vr::IVRPaths_Version, &eError);

              // The Master Clock: server_time_ticks
              WritePathProperty(pVRPaths, handle, "/server_time_ticks", tickTime);
              

              // Sequence and Size
              WritePathProperty(pVRPaths, handle, "/frame_sequence", (uint64_t)frameSequence++);
              WritePathProperty(pVRPaths, handle, "/frame_size", (int32_t)frameDataSize);

              // Timing and Rate
              double frameTime = frameSequence * (1.0 / 60.0);
              WritePathProperty(pVRPaths, handle, "/frame_time_monotonic", frameTime);
              WritePathProperty(pVRPaths, handle, "/delivery_rate", 60.0);
              WritePathProperty(pVRPaths, handle, "/elapsed_time", 1.0 / 60.0);

              // Convert and Release
              static BC4_to_NV12_Converter conv(TEXTURE_WIDTH, IMAGE_WIDTH, IMAGE_HEIGHT);
              conv.convert(data, data + BC4_DATA_SIZE, reinterpret_cast<uint8_t*>(blockImageBuffer));

              // Save to disk for debugging only once
              
              if (!hasSavedDebugImage) {
                // Use std::filesystem to get the temp directory in a cross-platform way
                std::filesystem::path tempDir = std::filesystem::temp_directory_path();
                std::filesystem::path debugImagePath = tempDir / "psvr2_debug_image.nv12";
                std::ofstream debugImageFile(debugImagePath, std::ios::binary);
                if (debugImageFile.is_open()) {
                  debugImageFile.write(reinterpret_cast<char*>(blockImageBuffer), frameDataSize);
                  debugImageFile.close();
                  Util::DriverLog("Saved debug image to {}", debugImagePath.string().c_str());
                }
                hasSavedDebugImage = true;
              }

              pVRBlockQueue->ReleaseWriteOnlyBlock(blockQueueHandle, handle);
          }
      }
  }

  // TODO: I believe false = failure, true = success?

  bool HmdDeviceCamera::GetCameraFrameDimensions(vr::ECameraVideoStreamFormat nVideoStreamFormat, uint32_t *pWidth, uint32_t *pHeight) {
    vr::VRDriverLog()->Log(__FUNCTION__);
    *pWidth = IMAGE_WIDTH;
    *pHeight = IMAGE_HEIGHT;
    return true;
  }

  bool HmdDeviceCamera::GetCameraFrameBufferingRequirements(int *pDefaultFrameQueueSize, uint32_t *pFrameBufferDataSize) {
    return false;
  }

  bool HmdDeviceCamera::SetCameraFrameBuffering(int nFrameBufferCount, void **ppFrameBuffers, uint32_t nFrameBufferDataSize) {
    return false;
  }

  bool HmdDeviceCamera::SetCameraVideoStreamFormat(vr::ECameraVideoStreamFormat nVideoStreamFormat) {
    return false;
  }

  vr::ECameraVideoStreamFormat HmdDeviceCamera::GetCameraVideoStreamFormat() {
    vr::VRDriverLog()->Log(__FUNCTION__);
    return vr::CVS_FORMAT_NV12;
  }

  bool HmdDeviceCamera::StartVideoStream() {
    vr::VRDriverLog()->Log(__FUNCTION__);
    // TODO: is this correct?
    shouldSubmit = true;
    return true;
  }

  void HmdDeviceCamera::StopVideoStream() {
    vr::VRDriverLog()->Log(__FUNCTION__);
    // TODO: is this correct?
    shouldSubmit = false;
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

  bool HmdDeviceCamera::SetAutoExposure(bool bEnable) {
    return false;
  }

  bool HmdDeviceCamera::PauseVideoStream() {
    vr::VRDriverLog()->Log(__FUNCTION__);
    // TODO: is this correct?
    shouldSubmit = false;
    return true;
  }

  bool HmdDeviceCamera::ResumeVideoStream() {
    vr::VRDriverLog()->Log(__FUNCTION__);
    // TODO: is this correct?
    shouldSubmit = true;

    hasSavedDebugImage = false;
    return true;
  }

  const float k_ZoomFactor = 3.25f;

  bool HmdDeviceCamera::GetCameraDistortion(uint32_t nCameraIndex, float flInputU, float flInputV, float *pflOutputU, float *pflOutputV) {
    flInputU *= IMAGE_WIDTH;
    flInputV *= IMAGE_HEIGHT;

    float out[2];

    if (nCameraIndex == 0) {
      DistortionParameters params{ {4.410233544294857, 8.984947311901681, -9.87816473467086e-7, 0.0000074754945954835176,
          3.677508810670342, 4.7174213801717, 10.2783500115958, 6.13988926548374,
          0, 0, 0, 0, -0.0015530241474945375, -0.00039289988739191575,
          0.32970695313327514, 0.005864799802305574, 0.000009239037037645196,
          0.971584734607621, 0.0367018715754216, 0.000227653240934961} };

      applyDistortionTransform(out, &params,
          ((flInputU - 505.8196716308594f) / 379.56439208984375f) * k_ZoomFactor,
          ((flInputV - 506.0033264160156f) / 379.56439208984375f) * k_ZoomFactor);

      *pflOutputU = ((out[0] * 379.56439208984375f) + 505.8196716308594f) / IMAGE_WIDTH;
      *pflOutputV = ((out[1] * 379.56439208984375f) + 506.0033264160156f) / IMAGE_HEIGHT;
    }
    else {
      DistortionParameters params{ {4.408362227825522,8.984405837950234,0.0000027455310754732335,-0.0000025145806185925286,
        3.679019420961825,4.7174213801717,10.2783500115958,6.13988926548374,0,0,0,0,-0.0017732032029994756,
        0.00023450291427025443,0.3299951208112014,0.005861164619598151,
        0.00000932997375226993,0.971584734607621,0.0367018715754216,0.000227653240934961} };

      applyDistortionTransform(out, &params,
        ((flInputU - 503.70745849609375f) / 381.2378234863281f) * k_ZoomFactor,
        ((flInputV - 504.4590759277344) / 381.2378234863281f) * k_ZoomFactor);

      *pflOutputU = ((out[0] * 381.2378234863281f) + 503.70745849609375f) / IMAGE_WIDTH;
      *pflOutputV = ((out[1] * 381.2378234863281f) + 504.4590759277344) / IMAGE_HEIGHT;
    }
    return true;
  }

  bool HmdDeviceCamera::GetCameraProjection(uint32_t nCameraIndex, vr::EVRTrackedCameraFrameType eFrameType, float flZNear, float flZFar, vr::HmdMatrix44_t *pProjection) {
    vr::VRDriverLog()->Log(__FUNCTION__);

    Util::DriverLog("camera: {}", nCameraIndex);

    if (nCameraIndex == 0) {
      // Camera properties from your code
      const float fy = 379.56439f;
      const float fx = 379.56439f;
      const float cx = 505.81967f;
      const float cy = 506.00332f;
      const float image_width = IMAGE_WIDTH * 2.0f;
      const float image_height = IMAGE_HEIGHT;

      // Convert pixel-space intrinsics to view-space projection plane boundaries at z_near
      // This defines the frustum shape that matches the camera sensor
      float left = -cx * (flZNear) / fx;
      float right = (image_width - cx) * (flZNear) / fx;
      float bottom = -(image_height - cy) * (flZNear) / fy;
      float top = cy * (flZNear) / fy;

      *pProjection = Util::createProjectionMatrix(flZNear, flZFar, left * k_ZoomFactor, right * k_ZoomFactor, top * k_ZoomFactor, bottom * k_ZoomFactor);
    }
    else {
      // Camera properties from your code
      const float fy = 381.23782f;
      const float fx = 381.23782f;
      const float cx = 503.70745f;
      const float cy = 504.45907f;
      const float image_width = IMAGE_WIDTH * 2.0f;
      const float image_height = IMAGE_HEIGHT;

      // Convert pixel-space intrinsics to view-space projection plane boundaries at z_near
      // This defines the frustum shape that matches the camera sensor
      float left = -cx * (flZNear) / fx;
      float right = (image_width - cx) * (flZNear) / fx;
      float bottom = -(image_height - cy) * (flZNear) / fy;
      float top = cy * (flZNear) / fy;
      *pProjection = Util::createProjectionMatrix(flZNear, flZFar, left * k_ZoomFactor, right * k_ZoomFactor, top * k_ZoomFactor, bottom * k_ZoomFactor);
    }

    //Util::SetIdentity(pProjection);

    return true;
  }

  bool HmdDeviceCamera::SetFrameRate(int nISPFrameRate, int nSensorFrameRate) {
    return false;
  }

  bool HmdDeviceCamera::SetCameraVideoSinkCallback(vr::ICameraVideoSinkCallback *pCameraVideoSinkCallback) {
    return false;
  }

  bool HmdDeviceCamera::GetCameraCompatibilityMode(vr::ECameraCompatibilityMode *pCameraCompatibilityMode) {
    return false;
  }

  bool HmdDeviceCamera::SetCameraCompatibilityMode(vr::ECameraCompatibilityMode nCameraCompatibilityMode) {
    return false;
  }

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
      *pWidth = IMAGE_WIDTH;
    }
    if (pHeight) {
      *pHeight = IMAGE_HEIGHT;
    }
    return true;
  }

  bool HmdDeviceCamera::GetCameraIntrinsics(
    uint32_t nCameraIndex, vr::EVRTrackedCameraFrameType eFrameType,
    vr::HmdVector2_t* pFocalLength, vr::HmdVector2_t* pCenter,
    vr::EVRDistortionFunctionType* peDistortionType,
    double rCoefficients[vr::k_unMaxDistortionFunctionParameters])
  {
    // Base hardware intrinsics
    float fx = nCameraIndex == 0 ? 379.56439f : 381.23782f;
    float fy = nCameraIndex == 0 ? 379.56439f : 381.23782f;
    float cx = nCameraIndex == 0 ? 505.81967f : 503.70745f;
    float cy = nCameraIndex == 0 ? 506.00332f : 504.45907f;

    if (eFrameType == vr::VRTrackedCameraFrameType_Distorted) {
      // Return raw physical parameters for the fisheye lens
      if (pFocalLength) {
        pFocalLength->v[0] = fx;
        pFocalLength->v[1] = fy;
      }
      if (pCenter) {
        pCenter->v[0] = cx;
        pCenter->v[1] = cy;
      }
      if (peDistortionType) *peDistortionType = vr::VRDistortionFunctionType_Extended_FTheta;

      // Populate rCoefficients here if requested...
    }
    else {
      // MaximumUndistorted or Undistorted frame
      // You widened the FOV in GetCameraProjection by k_ZoomFactor (3.25f).
      // The OpenXR layer needs to know the effective focal length of this zoomed frame.
      if (pFocalLength) {
        pFocalLength->v[0] = fx / k_ZoomFactor;
        pFocalLength->v[1] = fy / k_ZoomFactor;
      }

      // The center remains relative to the single-eye width (1024) which the OpenXR layer expects.
      if (pCenter) {
        pCenter->v[0] = cx;
        pCenter->v[1] = cy;
      }

      if (peDistortionType) *peDistortionType = vr::VRDistortionFunctionType_None;
      if (rCoefficients) {
        for (int i = 0; i < vr::k_unMaxDistortionFunctionParameters; ++i) rCoefficients[i] = 0.0;
      }
    }

    return true;
  }

}
