#include "driver_host_proxy.h"

#include "hmd_math.h"
#include "util.h"
#include "vr_settings.h"

#include <cstdint>

namespace psvr2_toolkit {

  /* Hardcoded device indexes, in order of when they are registered inside the driver. */

  static constexpr uint32_t k_unDeviceIndexHeadset = 0; // Currently not used.
  static constexpr uint32_t k_unDeviceIndexSenseControllerLeft = 1;
  static constexpr uint32_t k_unDeviceIndexSenseControllerRight = 2;

  DriverHostProxy *DriverHostProxy::m_pInstance = nullptr;

  DriverHostProxy::DriverHostProxy()
    : m_pDriverHost(nullptr)
    , m_pfnEventHandlers()
  {}
  
  DriverHostProxy *DriverHostProxy::Instance() {
    if (!m_pInstance) {
      m_pInstance = new DriverHostProxy;
    }

    return m_pInstance;
  }

  void DriverHostProxy::SetDriverHost(vr::IVRServerDriverHost *pDriverHost) {
    m_pDriverHost = pDriverHost;
  }

  void DriverHostProxy::AddEventHandler(void (*pfnEventHandler)(vr::VREvent_t *)) {
    m_pfnEventHandlers.push_back(pfnEventHandler);
  }

  bool DriverHostProxy::TrackedDeviceAdded(const char *pchDeviceSerialNumber, vr::ETrackedDeviceClass eDeviceClass, vr::ITrackedDeviceServerDriver *pDriver) {
    if (Util::StartsWith(pchDeviceSerialNumber, "playstation_vr2_sense_controller_") &&
        VRSettings::GetBool(STEAMVR_SETTINGS_DISABLE_SENSE, SETTING_DISABLE_SENSE_DEFAULT_VALUE))
    {
      return false;
    }

    return m_pDriverHost->TrackedDeviceAdded(pchDeviceSerialNumber, eDeviceClass, pDriver);
  }

  void DriverHostProxy::TrackedDevicePoseUpdated(uint32_t unWhichDevice, const vr::DriverPose_t &newPose, uint32_t unPoseStructSize) {
    if (unWhichDevice != k_unDeviceIndexSenseControllerLeft && unWhichDevice != k_unDeviceIndexSenseControllerRight) {
      return m_pDriverHost->TrackedDevicePoseUpdated(unWhichDevice, newPose, unPoseStructSize);
    }

    return m_pDriverHost->TrackedDevicePoseUpdated(unWhichDevice, GetPose(unWhichDevice, newPose), unPoseStructSize);
  }

  void DriverHostProxy::VsyncEvent(double vsyncTimeOffsetSeconds) {
    m_pDriverHost->VsyncEvent(vsyncTimeOffsetSeconds);
  }

  void DriverHostProxy::VendorSpecificEvent(uint32_t unWhichDevice, vr::EVREventType eventType, const vr::VREvent_Data_t &eventData, double eventTimeOffset) {
    m_pDriverHost->VendorSpecificEvent(unWhichDevice, eventType, eventData, eventTimeOffset);
  }

  bool DriverHostProxy::IsExiting() {
    return m_pDriverHost->IsExiting();
  }

  bool DriverHostProxy::PollNextEvent(vr::VREvent_t *pEvent, uint32_t uncbVREvent) {
    if (m_pDriverHost->PollNextEvent(pEvent, uncbVREvent)) {
      for (auto& m_pfnEventHandler : m_pfnEventHandlers) {
        m_pfnEventHandler(pEvent);
      }
      return true;
    }
    return false;
  }

  void DriverHostProxy::GetRawTrackedDevicePoses(float fPredictedSecondsFromNow, vr::TrackedDevicePose_t *pTrackedDevicePoseArray, uint32_t unTrackedDevicePoseArrayCount) {
    m_pDriverHost->GetRawTrackedDevicePoses(fPredictedSecondsFromNow, pTrackedDevicePoseArray, unTrackedDevicePoseArrayCount);
  }

  void DriverHostProxy::RequestRestart(const char *pchLocalizedReason, const char *pchExecutableToStart, const char *pchArguments, const char *pchWorkingDirectory) {
    m_pDriverHost->RequestRestart(pchLocalizedReason, pchExecutableToStart, pchArguments, pchWorkingDirectory);
  }

  uint32_t DriverHostProxy::GetFrameTimings(vr::Compositor_FrameTiming *pTiming, uint32_t nFrames) {
    return m_pDriverHost->GetFrameTimings(pTiming, nFrames);
  }

  void DriverHostProxy::SetDisplayEyeToHead(uint32_t unWhichDevice, const vr::HmdMatrix34_t &eyeToHeadLeft, const vr::HmdMatrix34_t &eyeToHeadRight) {
    m_pDriverHost->SetDisplayEyeToHead(unWhichDevice, eyeToHeadLeft, eyeToHeadRight);
  }

  void DriverHostProxy::SetDisplayProjectionRaw(uint32_t unWhichDevice, const vr::HmdRect2_t &eyeLeft, const vr::HmdRect2_t &eyeRight) {
    m_pDriverHost->SetDisplayProjectionRaw(unWhichDevice, eyeLeft, eyeRight);
  }

  void DriverHostProxy::SetRecommendedRenderTargetSize(uint32_t unWhichDevice, uint32_t nWidth, uint32_t nHeight) {
    m_pDriverHost->SetRecommendedRenderTargetSize(unWhichDevice, nWidth, nHeight);
  }

  vr::DriverPose_t DriverHostProxy::GetPose(uint32_t unWhichDevice, const vr::DriverPose_t &originalPose) {
    static vr::HmdQuaternion_t imuRotationOffset = HmdMath::EulerToQuaternion(0, 0, 0.680678427219391);
    static vr::HmdQuaternion_t imuRotationOffsetInverse = HmdMath::QuaternionInverse(imuRotationOffset);

    // Whether this is the left controller or not.
    bool isLeft = unWhichDevice == k_unDeviceIndexSenseControllerLeft;

    // Our new pose is a copy of the original pose.
    vr::DriverPose_t newPose = originalPose;

    // Apply inverse of imuRotationOffset to qRotation.
    newPose.qRotation = HmdMath::QuaternionMultiply(newPose.qRotation, imuRotationOffsetInverse);

    // PS VR2 driver pose offset.
    vr::HmdVector3d_t poseOffset = {isLeft ? 0.03439270332455635 : -0.03439270332455635, 0.05370872840285301, -0.09804324805736542};

    // Rotate the offset by the new rotation.
    vr::HmdVector3d_t rotationOffset = HmdMath::RotateVectorByQuaternion(poseOffset, newPose.qRotation);

    // Adjust position (negate the offset from the driver).
    newPose.vecPosition[0] -= rotationOffset.v[0];
    newPose.vecPosition[1] -= rotationOffset.v[1];
    newPose.vecPosition[2] -= rotationOffset.v[2];

    // Offset from the driver's root to the IMU. Given by the PS VR2 driver.
    // We'll also have to factor it to make the result pose identical to the one from the driver.
    vr::HmdVector3d_t imuOffset = {isLeft ? -0.00937270000576973 : 0.020072702318429947, 0.012248100712895393, 0.006003900431096554};

    // Rotate IMU offset to counteract the rotation we did on qRotation. See next comment.
    imuOffset = HmdMath::RotateVectorByQuaternion(imuOffset, imuRotationOffset);

    poseOffset.v[0] += imuOffset.v[0];
    poseOffset.v[1] += imuOffset.v[1];
    poseOffset.v[2] += imuOffset.v[2];

    newPose.vecDriverFromHeadTranslation[0] = poseOffset.v[0];
    newPose.vecDriverFromHeadTranslation[1] = poseOffset.v[1];
    newPose.vecDriverFromHeadTranslation[2] = poseOffset.v[2];

    // Since qRotation was rotated by the inverse of imuRotationOffset, we'll have to counteract it.
    newPose.qDriverFromHeadRotation = imuRotationOffset;

    return newPose;
  }

} // psvr2_toolkit
