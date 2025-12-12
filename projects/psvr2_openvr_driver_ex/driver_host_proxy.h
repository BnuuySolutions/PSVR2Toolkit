#pragma once

#include <openvr_driver.h>

#include <list>

namespace psvr2_toolkit {

  class DriverHostProxy : public vr::IVRServerDriverHost {
  public:
    DriverHostProxy();

    static DriverHostProxy *Instance();

    void SetDriverHost(vr::IVRServerDriverHost *pDriverHost);
    void AddEventHandler(void (*pfnEventHandler)(vr::VREvent_t *)); // Required for intercepting polled events from the PS VR2 driver.

    /** IVRServerDriverHost **/

    bool TrackedDeviceAdded(const char *pchDeviceSerialNumber, vr::ETrackedDeviceClass eDeviceClass, vr::ITrackedDeviceServerDriver *pDriver) override;
    void TrackedDevicePoseUpdated(uint32_t unWhichDevice, const vr::DriverPose_t &newPose, uint32_t unPoseStructSize) override;
    void VsyncEvent(double vsyncTimeOffsetSeconds) override;
    void VendorSpecificEvent(uint32_t unWhichDevice, vr::EVREventType eventType, const vr::VREvent_Data_t &eventData, double eventTimeOffset) override;
    bool IsExiting() override;
    bool PollNextEvent(vr::VREvent_t *pEvent, uint32_t uncbVREvent) override;
    void GetRawTrackedDevicePoses(float fPredictedSecondsFromNow, vr::TrackedDevicePose_t *pTrackedDevicePoseArray, uint32_t unTrackedDevicePoseArrayCount) override;
    void RequestRestart(const char *pchLocalizedReason, const char *pchExecutableToStart, const char *pchArguments, const char *pchWorkingDirectory) override;
    uint32_t GetFrameTimings(vr::Compositor_FrameTiming *pTiming, uint32_t nFrames) override;
    void SetDisplayEyeToHead(uint32_t unWhichDevice, const vr::HmdMatrix34_t &eyeToHeadLeft, const vr::HmdMatrix34_t &eyeToHeadRight) override;
    void SetDisplayProjectionRaw(uint32_t unWhichDevice, const vr::HmdRect2_t &eyeLeft, const vr::HmdRect2_t &eyeRight) override;
    void SetRecommendedRenderTargetSize(uint32_t unWhichDevice, uint32_t nWidth, uint32_t nHeight) override;

  private:
    static DriverHostProxy *m_pInstance;

    vr::IVRServerDriverHost *m_pDriverHost;
    std::list<void (*)(vr::VREvent_t *)> m_pfnEventHandlers;

    // Used internally for controller pose correction.
    vr::DriverPose_t GetPose(uint32_t unWhichDevice, const vr::DriverPose_t &originalPose);
  };

} // psvr2_toolkit
