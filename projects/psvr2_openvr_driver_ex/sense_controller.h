#pragma once

#include "math_helpers.h"
#include "libpad_hooks.h"
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
  uint8_t unkSettingsBit0 : 1;
  uint8_t rumbleEmulation : 1;
  uint8_t adaptiveTriggerSetEnable : 1;
  uint8_t intensityIncreaseSetEnable : 1; // Required to allow going from off to some level of intensity.
  uint8_t intensityReductionSetEnable : 1;
  uint8_t unkSettingsBit5 : 1;
  uint8_t unkSettingsBit6 : 1;
  uint8_t unkSettingsBit7 : 1;

  uint8_t unkSettingsBit8 : 1;
  uint8_t unkSettingsBit9 : 1;
  uint8_t statusLEDSetEnable : 1;
  uint8_t unkSettingsBit11 : 1;
  uint8_t unkSettingsBit12 : 1;
  uint8_t unkSettingsBit13 : 1;
  uint8_t unkSettingsBit14 : 1;
  uint8_t unkSettingsBit15 : 1;

  uint8_t rumbleIntensity;
  uint8_t unkData1; // Sony driver sometimes sets this to 0x82 for a report. Not sure what it does.
  SenseAdaptiveTriggerCommand_t adaptiveTriggerData;
  uint32_t timeStampMicrosecondsLastSend;
  SenseLED_t ledData;

  // From 0x0 (no reduction) to 0x7 (max reduction). Decreases in 12.5% increments.
  uint8_t hapticsIntensityReduction : 4;
  uint8_t triggerIntensityReduction : 4;

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

    void SetGeneratedHaptic(float freq, uint32_t amp, uint32_t sampleCount);
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

      // Reset filtered offset value to ensure everything is fresh
      this->filteredOffset = this->timeStampOffset;
    }

    double GetTimestampOffset() {
      std::scoped_lock<std::mutex> lock(this->controllerMutex);
      return this->filteredOffset + this->offsetLatency;
    }

    bool GetHasTimestampOffset() {
      std::scoped_lock<std::mutex> lock(this->controllerMutex);
      return this->hasTimestampOffset;
    }

    void AddTimestampOffsetSample(double sample) {
      std::scoped_lock<std::mutex> lock(this->controllerMutex);

      if (!this->hasTimestampOffset) {
        this->timeStampOffset = sample;
        this->filteredOffset = sample;
        this->hasTimestampOffset = true;
      }
      else {
        // Counter any potential drift to ensure accuracy.
        this->timeStampOffset -= (GetHostTimestamp() - this->lastSampleTimestamp) * 5.0E-5;

        if (this->timeStampOffset < sample) {
          this->timeStampOffset = sample;
        }

        // Update filtered offset with maximum delta
        double delta = this->timeStampOffset - this->filteredOffset;
        double maxDelta = Clamp(delta, -2.5, 2.5); // Increase or decrease by at most 2.5 microseconds per sample
        this->filteredOffset += maxDelta;
      }

      this->lastSampleTimestamp = GetHostTimestamp();
    }
    void ClearTimestampOffset() {
      std::scoped_lock<std::mutex> lock(this->controllerMutex);
      this->hasTimestampOffset = false;

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

    double timeStampOffset = 0.0;
    double filteredOffset = 0.0;
    bool hasTimestampOffset = false;
    uint64_t lastSampleTimestamp = 0;

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

    std::vector<int8_t> pcmData;
    size_t samplesRead = 0;
  };

  void StartSenseThread();
  void StopSenseThread();
}


