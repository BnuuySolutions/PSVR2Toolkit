#include "libpad_hooks.h"

#include "hmd_driver_loader.h"
#include "hook_lib.h"
#include "sense_controller.h"
#include "usb_thread_gaze.h"
#include "usb_thread_hooks.h"
#include "util.h"
#include "vr_settings.h"

namespace psvr2_toolkit {

    struct ProcessedControllerState {
        uint64_t hostReceiveTime;
        uint8_t  sequenceNumber;
        uint8_t  triggerFeedbackMode;
        uint8_t  unknown0x12[2];
        uint32_t buttons;
        uint8_t  leftStickX;
        uint8_t  leftStickY;
        uint8_t  rightStickX;
        uint8_t  rightStickY;
        uint8_t  leftTrigger;
        uint8_t  rightTrigger;
        uint16_t unknown0x1E;
        uint32_t sensorTimestamp;
        uint8_t  maybeTouchData;
        uint8_t  unknown0x25;
        int16_t  angularVelocityX;
        int16_t  angularVelocityZ;
        int16_t  angularVelocityY;
        int16_t  accelerometerX;
        int16_t  accelerometerY;
        int16_t  accelerometerZ;
        uint8_t  unknown0x32[0xC];
        uint8_t  leftTriggerFeedback;
        uint8_t  rightTriggerFeedback;
        uint8_t  triggerFeedbackLoc;
        uint8_t  unknown0x41[0x1B];
        uint8_t  isLeftController;
        uint8_t  proximityBits;
        uint8_t  triggerProximityAnalog;
        uint8_t  gripProximityAnalog;
        uint32_t deviceTimestamp;
        uint32_t loopbackTimestamp;
        uint8_t  leftBattery;
        uint8_t  rightBattery;
    };

    struct TimeSync
    {
        uint8_t unknown0x0[0x810];
        uint8_t isLeft;
        uint8_t unknown0x812[0x16f];
    }; static_assert(sizeof(TimeSync) == 0x980, "Size of TimeSync is not 0x980 bytes!");

	// This is slightly different from SenseLED_t, as this
    // is not packed and contains some additional data.
    struct LedSync
    {
        SenseLEDPhase phase;
        uint8_t seq;
        uint8_t period;
        int32_t baseTime;
        int32_t frameCycle;
        uint8_t leds[4];
        int64_t last_timestamp;
        bool unknown0x18; // Is LED sync data valid?
        int32_t camExposure;
        int32_t oneSubGridTime;
        int32_t oneSubGridTime_x15; // Unsure what this is, but this is oneSubGridTime times 15.
		bool unknown0x28; // Is camera data valid?
    }; static_assert(sizeof(LedSync) == 0x30, "Size of LedSync is not 0x30 bytes!");

    struct HidDeviceDescriptor
    {
        uint32_t unknown1[2];
        int padHandle;
        uint32_t unknown2;
        HANDLE controllerFileHandle;
    };

    const int k_libpadDeviceTypeSenseLeft = 3;
    const int k_libpadDeviceTypeSenseRight = 4;

    TimeStruct controllerTimings[2];

    const uint32_t k_unSenseUnitsPerMicrosecond = 3;

    // The device time that's passed in is already divided by 3.
    // This is so that all calcuations work within this bound.
    const uint32_t k_unDeviceTimestampMax = 0xFFFFFFFF / k_unSenseUnitsPerMicrosecond;

    // The total number of unique values in the device's clock cycle (its modulus).
    const uint32_t k_unDeviceTimestampModulus = k_unDeviceTimestampMax + 1;

    uintptr_t packetRecievedReturnAddress = 0;

    std::atomic<int32_t> latencyOffset = 0;

    inline const uint64_t getHostTimestamp()
    {
        static LARGE_INTEGER frequency{};
        if (frequency.QuadPart == 0)
        {
            QueryPerformanceFrequency(&frequency);
        }

        LARGE_INTEGER now;
        QueryPerformanceCounter(&now);

        // Return timestamp in microseconds.
        return static_cast<uint64_t>((static_cast<double>(now.QuadPart) /
            static_cast<double>(frequency.QuadPart)) * 1e6);
    }

    inline int32_t getWraparoundDifference(uint32_t newTimestamp, uint32_t oldTimestamp)
    {
        // Ensure inputs are within the device's clock domain.
        newTimestamp %= k_unDeviceTimestampModulus;
        oldTimestamp %= k_unDeviceTimestampModulus;

        // Calculate the direct forward difference.
        // We add k_unDeviceTimestampModulus before the final modulo to prevent underflow
        // if newTimestamp < oldTimestamp, ensuring the result is always positive.
        uint32_t forwardDiff = (newTimestamp - oldTimestamp + k_unDeviceTimestampModulus) % k_unDeviceTimestampModulus;

        // The shortest path is either forward or backward. If the forward path is more
        // than halfway around the clock, the backward path is shorter.
        if (forwardDiff > k_unDeviceTimestampModulus / 2)
        {
            // The backward difference is negative.
            // This is equivalent to `forwardDiff - MODULUS`.
            return static_cast<int32_t>(forwardDiff - k_unDeviceTimestampModulus);
        }
        else
        {
            // The forward difference is the shortest path.
            return static_cast<int32_t>(forwardDiff);
        }
    }

    uint32_t(*libpad_hostToDevice)(TimeSync* timeSync, uint32_t host, uint32_t* outDevice) = nullptr;
    uint32_t libpad_hostToDeviceHook(TimeSync* timeSync, uint32_t host, uint32_t* outDevice) {
        TimeStruct* timing = &controllerTimings[timeSync->isLeft ? 0 : 1];
        std::scoped_lock lock(timing->mutex);

        uint64_t calculatedTimestamp = static_cast<uint64_t>(host)
            + static_cast<uint64_t>(latencyOffset)
            + static_cast<uint64_t>(timing->timeStampOffset.getPercentile());

        *outDevice = static_cast<uint32_t>(calculatedTimestamp % k_unDeviceTimestampModulus);

        return 0;
    }

    uint32_t(*libpad_deviceToHost)(TimeSync* timeSync, uint32_t device, ProcessedControllerState* outHost) = nullptr;
    uint32_t libpad_deviceToHostHook(TimeSync* timeSync, uint32_t device, ProcessedControllerState* outHost) {
        TimeStruct* timing = &controllerTimings[timeSync->isLeft ? 0 : 1];
		std::scoped_lock lock(timing->mutex);

        uint64_t currentTime = getHostTimestamp();

        // We need a specific caller that actually passes ProcessedControllerState* into outHost.
        // This caller should be from when a controller packet is received.
        if (reinterpret_cast<uintptr_t>(_ReturnAddress()) == packetRecievedReturnAddress)
        {
            uint32_t deviceTimestamp = outHost->deviceTimestamp / k_unSenseUnitsPerMicrosecond;

            int32_t clockOffset = getWraparoundDifference(deviceTimestamp, static_cast<uint32_t>(currentTime));

            timing->timeStampOffset.add(static_cast<double>(clockOffset));
        }

        // Translate the device timestamp back to the host's 64-bit time domain.
        uint32_t hostTime = (static_cast<uint32_t>(device)
                            - static_cast<uint32_t>(timing->timeStampOffset.getPercentile())
                            + k_unDeviceTimestampModulus) % k_unDeviceTimestampModulus;

        int32_t difference = getWraparoundDifference(hostTime, static_cast<uint32_t>(currentTime)) - latencyOffset;

        outHost->hostReceiveTime = currentTime + difference;

        return 0;
    }

    HidDeviceDescriptor* (*libpad_CreateHidDevice)(HidDeviceDescriptor* device, const wchar_t* name, int deviceType) = nullptr;
    HidDeviceDescriptor* libpad_CreateHidDeviceHook(HidDeviceDescriptor* device, const wchar_t* name, int deviceType) {
        Util::DriverLog("CreateHidDeviceHook called for device: %ls, type: %d", name, deviceType);

		HidDeviceDescriptor* result = libpad_CreateHidDevice(device, name, deviceType);

        if (deviceType == k_libpadDeviceTypeSenseLeft)
        {
            SenseController::GetLeftController().SetHandle(device->controllerFileHandle, result->padHandle);
            controllerTimings[0].timeStampOffset.clear();
        }
        else if (deviceType == k_libpadDeviceTypeSenseRight)
        {
            SenseController::GetRightController().SetHandle(device->controllerFileHandle, result->padHandle);
            controllerTimings[1].timeStampOffset.clear();
        }

        return result;
    }

    void (*libpad_Disconnect)(int device) = nullptr;
    void libpad_DisconnectHook(int device) {
        Util::DriverLog("libpadDisconnectHook called for device handle: %d", device);

        try {
            SenseController& controller = SenseController::GetControllerByPadHandle(device);
            controller.SetHandle(NULL, -1);
        }
        catch (const std::runtime_error&) {
            // No controller with the given pad handle.
		}

        libpad_Disconnect(device);
    }

    int (*libpad_SendOutputReport)(int device, const unsigned char* buffer, unsigned short size) = nullptr;
    int libpad_SendOutputReportHook(int device, const unsigned char* buffer, unsigned short size) {
        if (size != sizeof(SenseControllerPCModePacket_t)) {
			Util::DriverLog("libpad_SendOutputReportHook called with unexpected size: %d", size);
			return libpad_SendOutputReport(device, buffer, size);
		}

        try {
            SenseController& controller = SenseController::GetControllerByPadHandle(device);
			controller.SetTrackingControllerSettings(reinterpret_cast<const SenseControllerPCModePacket_t*>(buffer));
        }
        catch (const std::runtime_error&) {
            // No controller with the given pad handle.
            return 0x8001002b;
        }

        return 0;
    }

    void (*libpad_SetSyncLedBaseTime)(TimeSync* timeSync, LedSync* ledSync) = nullptr;
    void libpad_SetSyncLedBaseTimeHook(TimeSync* timeSync, LedSync* ledSync) {
        static std::mutex ledSyncMutex;
        std::scoped_lock lock(ledSyncMutex);

		int controller = timeSync->isLeft ? 0 : 1;
        int samples;

        {
            TimeStruct* timing = &controllerTimings[controller];
            std::scoped_lock lock(timing->mutex);
            samples = static_cast<int>(timing->timeStampOffset.size());
        }

        static int currentController = -1;
        static int ledSyncPart = 0;
		static int lastLedCount = 0;
        static uint64_t timeout = getHostTimestamp();
        static uint64_t lastSync = getHostTimestamp();

        // We only want one controller to run this logic.
        if (currentController == -1) {
            currentController = controller;
        }

        int currentLedCount = currentLDPayload.cameras[0].num_leds
            + currentLDPayload.cameras[1].num_leds
            + currentLDPayload.cameras[2].num_leds
            + currentLDPayload.cameras[3].num_leds;

        // TODO: calibration should be triggered via IPC
        if (GetAsyncKeyState(VK_F9) & 0x8000) {
            latencyOffset = 0;
            ledSyncPart = 0;
            currentController = -1;

            ledSync->seq++;
            timeout = getHostTimestamp();
        }

		bool needsCalibration = (ledSyncPart < 4);

        if (needsCalibration)
        {
            ledSync->last_timestamp -= 10000000; // Force the update.
        }

		// Before we start calibration, we need to
		// - Check that we need calibration.
        // - Ensure there are enough samples for getting the time offset.
		// - Ensure we have waited at least 1 second since the last calibration step.
		// - Ensure this is the controller that is doing the calibration.
        if (needsCalibration
            && samples > 500
            && getHostTimestamp() - timeout > 1000000
            && currentController == controller) {
			// Force to PRESCAN phase while calibrating.
            ledSync->phase = PRESCAN;
            ledSync->period = 6;

			uint32_t fullWindow = (ledSync->oneSubGridTime * static_cast<int32_t>(ledSync->period)) + ledSync->camExposure;

            if (getHostTimestamp() - lastSync > 150000)
            {
                switch (ledSyncPart)
                {
                case 0:
                    latencyOffset += fullWindow / 3;

                    // We want to see an increase in the number of LEDs tracked. (right edge)
                    if (currentLedCount > lastLedCount) {
                        ledSyncPart++;
                    }

                    break;
                case 1:
                    latencyOffset -= fullWindow / 9;

                    // We want to see a decrease in the number of LEDs tracked. (left edge)
                    if (currentLedCount <= lastLedCount) {
                        ledSyncPart++;
                    }

                    break;
                case 2:
                    latencyOffset += fullWindow / 16;

                    // We want to see an increase in the number of LEDs tracked. (right edge)
                    if (currentLedCount > lastLedCount) {
                        ledSyncPart++;
                    }
                    break;
                case 3:
                    latencyOffset -= fullWindow / 48;

                    // We want to see a decrease in the number of LEDs tracked.
                    // At this point, we should be on the edge by 10 microseconds.
                    if (currentLedCount <= lastLedCount) {
                        ledSyncPart++;

                        // Go to the center of this cycle, and then we now have our correct latency offset.
                        latencyOffset += fullWindow / 2;

                        Util::DriverLog("Completed latency calibration. Final latency offset: %d microseconds", latencyOffset.load());
                    }
                    break;
                default:
                    break;
                }

                Util::DriverLog("Latency Offset is currently at %d microseconds.", latencyOffset.load());

				lastSync = getHostTimestamp();
            }
		}
        else if (needsCalibration) {
			// Ensure LEDs are off before we start calibration,
            // or if we're not the controller doing calibration.
			ledSync->phase = LED_ALL_OFF;
            ledSync->seq++;

            if (currentController == controller)
            {
                // Set baseline LED count. The blob tracking may pick up other sources.
                // Also add a small margin of error.
                lastLedCount = currentLedCount + 12;
            }
        }
        else if (getHostTimestamp() - lastSync < 1000000)
        {
            // If calibration finished recently, we need to make sure the controllers both go back into prescan mode.
            ledSync->phase = PRESCAN;
        }

        if (latencyOffset < 0 || latencyOffset > 16666)
        {
            latencyOffset = 0;
            ledSyncPart = 0;
        }

		// This call uses libpad_hostToDevice, which will factor in the updated latencyOffset.
		libpad_SetSyncLedBaseTime(timeSync, ledSync);
    }

    void LibpadHooks::InstallHooks() {
        static HmdDriverLoader* pHmdDriverLoader = HmdDriverLoader::Instance();

        if (VRSettings::GetBool(STEAMVR_SETTINGS_USE_TOOLKIT_SYNC, SETTING_USE_TOOLKIT_SYNC_DEFAULT_VALUE)) {
            Util::DriverLog("Using custom controller/LED sync...");

            packetRecievedReturnAddress = pHmdDriverLoader->GetBaseAddress() + 0x1c75a1;

			// libpad function for uint32_t libpad_hostToDevice(TimeSync* timeSync, uint32_t host, uint32_t* outDevice) @ 0x1C09E0
            HookLib::InstallHook(reinterpret_cast<void*>(pHmdDriverLoader->GetBaseAddress() + 0x1C09E0),
                reinterpret_cast<void*>(libpad_hostToDeviceHook),
                reinterpret_cast<void**>(&libpad_hostToDevice));

			// libpad function for uint32_t libpad_deviceToHost(TimeSync* timeSync, uint32_t device, ProcessedControllerState* outHost) @ 0x1C1520
            HookLib::InstallHook(reinterpret_cast<void*>(pHmdDriverLoader->GetBaseAddress() + 0x1C1520),
                reinterpret_cast<void*>(libpad_deviceToHostHook),
                reinterpret_cast<void**>(&libpad_hostToDevice));

            // libpad function for void libpad_SetSyncLedBaseTime(TimeSync* timeSync, LedSync* ledSync) @ 0x1C27D0
            HookLib::InstallHook(reinterpret_cast<void*>(pHmdDriverLoader->GetBaseAddress() + 0x1C27D0),
                reinterpret_cast<void*>(libpad_SetSyncLedBaseTimeHook),
                reinterpret_cast<void**>(&libpad_SetSyncLedBaseTime));
        }

        if (VRSettings::GetBool(STEAMVR_SETTINGS_USE_ENHANCED_HAPTICS, SETTING_USE_TOOLKIT_SYNC_DEFAULT_VALUE)) {
            // libpad function for int CreateHidDevice(HidDeviceDescriptor* device, wchar_t name, int deviceType) @ 0x1CE210
            HookLib::InstallHook(reinterpret_cast<void*>(pHmdDriverLoader->GetBaseAddress() + 0x1CE210),
                reinterpret_cast<void*>(libpad_CreateHidDeviceHook),
                reinterpret_cast<void**>(&libpad_CreateHidDevice));

            // libpad function for void libpad_Disconnect(int device) @ 0x1C7E90
            HookLib::InstallHook(reinterpret_cast<void*>(pHmdDriverLoader->GetBaseAddress() + 0x1C7E90),
                reinterpret_cast<void*>(libpad_DisconnectHook),
                reinterpret_cast<void**>(&libpad_Disconnect));

            // libpad function for int libpad_SendOutputReport(int handle, uchar * buffer, ushort size) @ 0x1CBA20
            HookLib::InstallHook(reinterpret_cast<void*>(pHmdDriverLoader->GetBaseAddress() + 0x1CBA20),
                reinterpret_cast<void*>(libpad_SendOutputReportHook),
                reinterpret_cast<void**>(&libpad_SendOutputReport));
        }
    }

} // psvr2_toolkit
