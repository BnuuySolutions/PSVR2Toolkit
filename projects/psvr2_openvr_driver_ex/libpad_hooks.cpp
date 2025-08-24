#include "libpad_hooks.h"

#include "hmd_driver_loader.h"
#include "hook_lib.h"
#include "rolling_percentile.h"
#include "usb_thread_gaze.h"
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
        char unknown0x0[0x810];
        char isLeft;
        char unknown0x812[0x16f];
    }; static_assert(sizeof(TimeSync) == 0x980, "Size of TimeSync is not 0x980 bytes!");

    struct TimeStruct
    {
        RollingPercentile<double> timeStampOffset = RollingPercentile<double>(5000, 99.0);

        uint32_t lastDeviceTimestamp = 0;
        uint32_t lastLoopbackTimestamp = 0;
    };

    TimeStruct controllerTimings[2];

    const uint32_t k_unSenseUnitsPerMicrosecond = 3;

    // The device time that's passed in is already divided by 3.
    // This is so that all calcuations work within this bound.
    const uint32_t k_unDeviceTimestampMax = 0xFFFFFFFF / k_unSenseUnitsPerMicrosecond;

    // The total number of unique values in the device's clock cycle (its modulus).
    const uint32_t k_unDeviceTimestampModulus = k_unDeviceTimestampMax + 1;

    uintptr_t packetRecievedReturnAddress = 0;

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

        uint64_t calculatedTimestamp = static_cast<uint64_t>(host) + static_cast<uint32_t>(timing->timeStampOffset.getPercentile());

        *outDevice = static_cast<uint32_t>(calculatedTimestamp % k_unDeviceTimestampModulus);

        return 0;
    }

    uint32_t(*libpad_deviceToHost)(TimeSync* timeSync, uint32_t device, ProcessedControllerState* outHost) = nullptr;
    uint32_t libpad_deviceToHostHook(TimeSync* timeSync, uint32_t device, ProcessedControllerState* outHost) {
        TimeStruct* timing = &controllerTimings[timeSync->isLeft ? 0 : 1];
        uint64_t currentTime = getHostTimestamp();

        // We need a specific caller that actually passes ProcessedControllerState* into outHost.
        // This caller should be from when a controller packet is received.
        if (reinterpret_cast<uintptr_t>(_ReturnAddress()) == packetRecievedReturnAddress)
        {
            uint32_t deviceTimestamp = outHost->deviceTimestamp / k_unSenseUnitsPerMicrosecond;

            // TODO: Figure out where this offset comes from.
			// Probably the real latency between host and device.
            uint32_t offsetInMicroseconds = 2500;
            int32_t clockOffset = getWraparoundDifference(deviceTimestamp + offsetInMicroseconds, static_cast<uint32_t>(currentTime));

            timing->timeStampOffset.add(static_cast<double>(clockOffset));
        }

        // Translate the device timestamp back to the host's 64-bit time domain.
        uint32_t hostTime = (static_cast<uint32_t>(device) - static_cast<uint32_t>(timing->timeStampOffset.getPercentile()) + k_unDeviceTimestampModulus) % k_unDeviceTimestampModulus;

        int32_t difference = getWraparoundDifference(hostTime, static_cast<uint32_t>(currentTime));

        outHost->hostReceiveTime = currentTime + difference;

        return 0;
    }

    void LibpadHooks::InstallHooks() {
        static HmdDriverLoader* pHmdDriverLoader = HmdDriverLoader::Instance();

        if (VRSettings::GetBool(STEAMVR_SETTINGS_USE_CUSTOM_SYNC, SETTING_USE_CUSTOM_SYNC_DEFAULT_VALUE)) {
            Util::DriverLog("Using custom controller/LED sync...");

            packetRecievedReturnAddress = pHmdDriverLoader->GetBaseAddress() + 0x1c75a1;

            // libpad function for host to device time domain conversion
            HookLib::InstallHook(reinterpret_cast<void*>(pHmdDriverLoader->GetBaseAddress() + 0x1C09E0),
                reinterpret_cast<void*>(libpad_hostToDeviceHook),
                reinterpret_cast<void**>(&libpad_hostToDevice));

            // libpad function for device to host time domain conversion
            HookLib::InstallHook(reinterpret_cast<void*>(pHmdDriverLoader->GetBaseAddress() + 0x1C1520),
                reinterpret_cast<void*>(libpad_deviceToHostHook),
                reinterpret_cast<void**>(&libpad_hostToDevice));
        }
    }

} // psvr2_toolkit
