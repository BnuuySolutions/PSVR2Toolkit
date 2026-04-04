#pragma once

#include <openvr_driver.h>
#include "openvr_driver_internal.h"
#include <thread>
#include <queue>
#include <mutex>

const int IMAGE_WIDTH = 1016;
const int IMAGE_HEIGHT = 1016;

// Texture is 1024 by 1016, but the real image is actually 1016x1016
const int TEXTURE_WIDTH = 1024;
const int TEXTURE_HEIGHT = 1016;

const size_t BC4_DATA_SIZE = (TEXTURE_WIDTH * TEXTURE_HEIGHT) / 2; // BC4 is 4 bits per pixel

const int32_t pixelCount = IMAGE_WIDTH * 2 * IMAGE_WIDTH;
const int32_t frameDataSize = pixelCount * 3 / 2; // NV12 is 1.5 bytes per pixel

namespace psvr2_toolkit {

  class HmdDeviceCamera : public vr::IVRCameraComponent {
  public:
    static HmdDeviceCamera *Instance();
    void UploadBC4(uint64_t tickTime, uint8_t* data);

    vr::IVRBlockQueue* pVRBlockQueue = nullptr;
    vr::PropertyContainerHandle_t blockQueueHandle;
    bool shouldSubmit = false;
    uint64_t frameSequence = 0;

    /** IVRCameraComponent **/

    bool GetCameraFrameDimensions(vr::ECameraVideoStreamFormat nVideoStreamFormat, uint32_t *pWidth, uint32_t *pHeight) override;
    bool GetCameraFrameBufferingRequirements(int *pDefaultFrameQueueSize, uint32_t *pFrameBufferDataSize) override;
    bool SetCameraFrameBuffering(int nFrameBufferCount, void **ppFrameBuffers, uint32_t nFrameBufferDataSize) override;
    bool SetCameraVideoStreamFormat(vr::ECameraVideoStreamFormat nVideoStreamFormat) override;
    vr::ECameraVideoStreamFormat GetCameraVideoStreamFormat() override;
    bool StartVideoStream() override;
    void StopVideoStream() override;
    bool IsVideoStreamActive(bool *pbPaused, float *pflElapsedTime) override;
    const vr::CameraVideoStreamFrame_t *GetVideoStreamFrame() override;
    void ReleaseVideoStreamFrame(const vr::CameraVideoStreamFrame_t *pFrameImage) override;
    bool SetAutoExposure(bool bEnable) override;
    bool PauseVideoStream() override;
    bool ResumeVideoStream() override;
    bool GetCameraDistortion(uint32_t nCameraIndex, float flInputU, float flInputV, float *pflOutputU, float *pflOutputV) override;
    bool GetCameraProjection(uint32_t nCameraIndex, vr::EVRTrackedCameraFrameType eFrameType, float flZNear, float flZFar, vr::HmdMatrix44_t *pProjection) override;
    bool SetFrameRate(int nISPFrameRate, int nSensorFrameRate) override;
    bool SetCameraVideoSinkCallback(vr::ICameraVideoSinkCallback *pCameraVideoSinkCallback) override;
    bool GetCameraCompatibilityMode(vr::ECameraCompatibilityMode *pCameraCompatibilityMode) override;
    bool SetCameraCompatibilityMode(vr::ECameraCompatibilityMode nCameraCompatibilityMode) override;
    bool GetCameraFrameBounds(vr::EVRTrackedCameraFrameType eFrameType, uint32_t *pLeft, uint32_t *pTop, uint32_t *pWidth, uint32_t *pHeight) override;
    bool GetCameraIntrinsics(uint32_t nCameraIndex, vr::EVRTrackedCameraFrameType eFrameType, vr::HmdVector2_t *pFocalLength, vr::HmdVector2_t *pCenter, vr::EVRDistortionFunctionType *peDistortionType, double rCoefficients[vr::k_unMaxDistortionFunctionParameters]) override;
  };

} // psvr2_toolkit

extern psvr2_toolkit::HmdDeviceCamera *g_pHmdDeviceCamera;
