#pragma once

#include "math_helpers.h"
#include "libpad_hooks.h"
#include "rolling_percentile.h"
#include "write_file_async.h"

#include <atomic>
#include <cmath>
#include <mutex>
#include <vector>
#include <string>

constexpr uint32_t k_unSenseSampleRate = 3000;
constexpr uint32_t k_unSenseSubsamples = 10000;
constexpr uint32_t k_unSenseMaxSamplePosition = k_unSenseSampleRate * k_unSenseSubsamples;
constexpr uint32_t k_unSenseHalfSamplePosition = k_unSenseMaxSamplePosition / 2;
constexpr uint8_t k_unSenseMaxHapticAmplitude = 127;

constexpr uint32_t k_unSenseUnitsPerMicrosecond = 3;

// Do not apply alignment
#pragma pack(push, 1)
struct SenseAdaptiveTriggerCommand_t {
    uint8_t mode;
    uint8_t commandData[10];
};

struct SenseResponse_t {
    uint8_t unkData1[29];
    uint32_t timeStampMicrosecondsLastSend;
	uint8_t unkData2[12];
	uint32_t inputReportMicroseconds;
    uint32_t timeStampMicrosecondsCurrent;
    uint8_t unkData3[25];
}; static_assert(sizeof(SenseResponse_t) == 78, "Size of SenseResponse_t is not 78 bytes!");

struct SenseLED_t {
    uint8_t phase;
    uint8_t sequenceNumber; // Increment when this struct changes
    uint8_t periodId;
    uint32_t cyclePosition; // For PRESCAN, this sets the position in the cycle, for other phases this is an offset from the existing cycle position.
    uint32_t cycleLength; // Length of the cycle
	uint8_t ledBlink[4]; // Unsure of what this is used for, but it looks like it is used for blinking the LEDs in a certain pattern.
}; static_assert(sizeof(SenseLED_t) == 15, "Size of SenseLED_t is not 15 bytes!");

struct SenseControllerSettings_t {
    uint8_t unkData1[4];
	SenseAdaptiveTriggerCommand_t adaptiveTriggerData;
    uint32_t timeStampMicrosecondsLastSend;
    SenseLED_t ledData;
    uint8_t hapticMode;
    bool ledEnable;
    uint8_t unkData3;
    uint8_t unkData4;
}; static_assert(sizeof(SenseControllerSettings_t) == 38, "Size of SenseControllerSettings_t is not 38 bytes!");

// This assumes mode 0xa0.
struct SenseControllerPacket_t {
    uint8_t reportId;
	uint8_t mode; // This does seem to change the layout of the struct depending on how this is set.
	uint8_t unkData1; // Enables or disables some stuff, but doesn't seem to change the layout?
	SenseControllerSettings_t settings;
	uint8_t packetNum; // Incremented every time we send a packet to this controller.
	uint8_t hapticPCM[32]; // 3000hz, 8 bit signed PCM data. This means we must send PCM packets at a rate of 93.75 times per second.
	uint8_t crc[4]; // uint8_t for alignment
}; static_assert(sizeof(SenseControllerPacket_t) == 78, "Size of SenseControllerPacket_t is not 78 bytes!");

// This assumes mode 0xa2.
struct SenseControllerPCModePacket_t {
    uint8_t mode; // This does seem to change the layout of the struct depending on how this is set.
    SenseControllerSettings_t settings;
    uint8_t padding[9];
}; static_assert(sizeof(SenseControllerPCModePacket_t) == 0x30, "Size of SenseControllerPacket_t is not 78 bytes!");
#pragma pack(pop)

// The official driver calculates the host timestamp this way.
// It simply converts QueryPerformanceCounter to unsigned 64bit microseconds.
inline const uint64_t GetHostTimestamp()
{
    static LARGE_INTEGER frequency{};
    if (frequency.QuadPart == 0)
    {
        QueryPerformanceFrequency(&frequency);
    }

    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);

    return static_cast<uint64_t>((static_cast<double>(now.QuadPart) /
        static_cast<double>(frequency.QuadPart)) * 1e6);
}

namespace psvr2_toolkit {
    class SenseController {
    public:
        SenseController(bool isLeft) : isLeft(isLeft) {}
        ~SenseController() = default;

        static void Initialize();
        static void Destroy();

        void SetGeneratedHaptic(float freq, uint32_t amp, uint32_t sampleCount, bool phaseJump);
        void SetPCM(const std::vector<int8_t>& newPCMData);
        void AppendPCM(const std::vector<int8_t>& newPCMData);

        const SenseControllerPCModePacket_t& GetTrackingControllerSettings() { return driverTrackingData; };

        void SetTrackingControllerSettings(const SenseControllerPCModePacket_t* data);
        void SetAdaptiveTriggerData(const SenseAdaptiveTriggerCommand_t* data);

        void SetIsTracking(bool tracking, uint64_t timestamp) {
            std::scoped_lock<std::mutex> lock(this->controllerMutex);
            this->isTracking = tracking;
            if (tracking) {
                this->lastTrackedTimestamp = timestamp;
			}
        }
        bool GetTrackingState(uint64_t& outLastTrackedTimestamp) {
            std::scoped_lock<std::mutex> lock(this->controllerMutex);
            outLastTrackedTimestamp = this->lastTrackedTimestamp;
			return this->isTracking;
        }
        void SetLibpadSyncs(LibpadTimeSync* timeSync, LibpadLedSync* ledSync) {
			std::scoped_lock<std::mutex> lock(this->controllerMutex);
			this->timeSync = timeSync;
			this->ledSync = ledSync;
        }
        void GetLibpadSyncs(LibpadTimeSync*& outTimeSync, LibpadLedSync*& outLedSync) {
            std::scoped_lock<std::mutex> lock(this->controllerMutex);
            outTimeSync = this->timeSync;
			outLedSync = this->ledSync;
        }

        int32_t GetLatencyOffset() {
            std::scoped_lock<std::mutex> lock(this->controllerMutex);
            return this->offsetLatency;
        }

        void SetLatencyOffset(int32_t newOffsetLatency) {
            std::scoped_lock<std::mutex> lock(this->controllerMutex);
            this->offsetLatency = newOffsetLatency;
        }

        double GetTimestampOffset() {
            std::scoped_lock<std::mutex> lock(this->controllerMutex);
            return this->timeStampOffset.getPercentile() + this->offsetLatency;
		}
        size_t GetTimestampOffsetSampleCount() {
            std::scoped_lock<std::mutex> lock(this->controllerMutex);
            return this->timeStampOffset.size();
        }
        void AddTimestampOffsetSample(double sample) {
            std::scoped_lock<std::mutex> lock(this->controllerMutex);
            this->timeStampOffset.add(sample);
		}
        void ClearTimestampOffsetSamples() {
            std::scoped_lock<std::mutex> lock(this->controllerMutex);
            this->timeStampOffset.clear();

            // Also reset offset for latency
            this->offsetLatency = -1;
        }

        const void* GetHandle();
        void SetHandle(void* handle, int padHandle);

        void SendToDevice();

        static SenseController& GetLeftController() { return leftController; }
        static SenseController& GetRightController() { return rightController; }

        static SenseController& GetControllerByPadHandle(int padHandle) {
			std::scoped_lock<std::mutex, std::mutex> lock1(leftController.controllerMutex, rightController.controllerMutex);

            if (leftController.padHandle == padHandle) {
                return leftController;
            }
            else if (rightController.padHandle == padHandle) {
                return rightController;
            }
            throw std::runtime_error("No controller with the given pad handle.");
		}

        static SenseController& GetControllerByIsLeft(bool isLeft) {
            return isLeft ? leftController : rightController;
		}

        const bool isLeft;

        // Should visible LEDs on the controllers be on or off?
        // This should be set to false to save power when the user is wearing the headset and doesn't need to see the LEDs.
        static std::atomic<bool> g_StatusLED;

        static std::atomic<uint8_t> g_ShouldResetLEDTrackingInTicks;
    private:
        void* handle = NULL;
		int padHandle = -1;
        std::mutex controllerMutex;

        RollingPercentile<double> timeStampOffset = RollingPercentile<double>(5000, 99.0);

        bool isTracking = false;
        uint64_t lastTrackedTimestamp = 0;

        int32_t offsetLatency = -1;

        LibpadTimeSync* timeSync = nullptr;
        LibpadLedSync* ledSync = nullptr;

        AsyncFileWriter asyncWriter;

        SenseControllerPCModePacket_t driverTrackingData = {};
        SenseAdaptiveTriggerCommand_t adaptiveTriggerData = {};

        static SenseController leftController;
        static SenseController rightController;

        uint32_t lastDeviceTimestamp = 0;
        uint32_t lastLoopbackTimestamp = 0;

        uint8_t hapticPacketIncrement = 0;

        uint32_t hapticPosition = 0;
        uint32_t hapticSamplesLeft = 0;
        uint32_t hapticAmp = 0;
        float hapticFreq = 0.0f;
        bool phaseJump = true;
        
        std::vector<int8_t> pcmData;
        size_t samplesRead = 0;
    };

    void StartSenseThread();
    void StopSenseThread();
}


