#include "driver_host_proxy.h"
#include "hmd_driver_loader.h"
#include "math_helpers.h"
#include "sense_controller.h"
#include "sense_crc.h"

#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>

#include <timeapi.h>
#include <windows.h>

#pragma comment(lib, "winmm.lib")

using namespace psvr2_toolkit;

std::atomic<bool> SenseController::g_StatusLED = true;
std::atomic<uint8_t> SenseController::g_ShouldResetLEDTrackingInTicks = 0;
SenseController SenseController::leftController = SenseController(true);
SenseController SenseController::rightController = SenseController(false);

std::atomic<std::thread*> hapticsThread;

void SenseController::SetGeneratedHaptic(float freq, uint32_t amp, uint32_t sampleCount, bool phaseJump) {
  std::scoped_lock<std::mutex> lock(controllerMutex);

  this->hapticFreq = freq;
  this->hapticAmp = amp;
  this->phaseJump = phaseJump;
  this->hapticSamplesLeft = sampleCount;
}
void SenseController::SetPCM(const std::vector<int8_t>& newPCMData) {
  std::scoped_lock<std::mutex> lock(controllerMutex);

  this->pcmData = newPCMData;
  this->samplesRead = 0;
}
void SenseController::AppendPCM(const std::vector<int8_t>& newPCMData) {
  std::scoped_lock<std::mutex> lock(controllerMutex);

  // Append the new PCM data to the existing data
  this->pcmData.insert(pcmData.end(), newPCMData.begin(), newPCMData.end());

  // Trim off data that is already read
  // Move current head to beginning of the vector
  std::move(pcmData.begin() + this->samplesRead, pcmData.end(), pcmData.begin());
  this->pcmData.resize(pcmData.size() - this->samplesRead);
  this->samplesRead = 0;
}

void SenseController::SetTrackingControllerSettings(const SenseControllerPCModePacket_t* data) {
  std::scoped_lock<std::mutex> lock(controllerMutex);

  if (data->settings.adaptiveTriggerSetEnable)
  {
    memcpy(&this->adaptiveTriggerData, &data->settings.adaptiveTriggerData, sizeof(SenseAdaptiveTriggerCommand_t));
  }

  memcpy(&this->driverTrackingData, data, sizeof(SenseControllerPCModePacket_t));
}

void SenseController::SetAdaptiveTriggerData(const SenseAdaptiveTriggerCommand_t* data) {
  std::scoped_lock<std::mutex> lock(controllerMutex);

  memcpy(&this->adaptiveTriggerData, data, sizeof(SenseAdaptiveTriggerCommand_t));
}

const void* SenseController::GetHandle() {
  std::scoped_lock<std::mutex> lock(controllerMutex);

  return this->handle;
}

void SenseController::SetHandle(void* handle, int padHandle) {
  {
    std::scoped_lock<std::mutex> lock(controllerMutex);

    this->handle = handle;
    this->padHandle = padHandle;
  }

  if (handle != nullptr)
  {
    this->SetGeneratedHaptic(800.0f, k_unSenseMaxHapticAmplitude, 1500, false);
    this->ClearTimestampOffset();
  }
}

uint32_t offset = 0;

void SenseController::SendToDevice() {
  SenseControllerPacket_t buffer;

  {
    std::scoped_lock<std::mutex> lock(this->controllerMutex);

    if (this->handle == NULL)
    {
      return;
    }

    memset(&buffer, 0, sizeof(buffer));

    buffer.reportId = 0x31;
    buffer.mode = 0xa0;
    buffer.unkData1 = 0x10;

    uint32_t* crc = (uint32_t*)(&buffer.crc);

    // With the 0xa0 mode, the rest of the data past the first few bytes are shifted ahead by one byte.
    // So copy what we did starting from the 5th byte in the original buffer to the 6th byte in the new buffer.
    memcpy(&buffer.settings, &this->driverTrackingData.settings, sizeof(SenseControllerSettings_t));

    // Disallow normal haptics, we are using PCM haptics.
    buffer.settings.rumbleEmulation = 0;

    // Always enable adaptive triggers
    buffer.settings.adaptiveTriggerSetEnable = 1;

    // Always enable intensity reduction and increase
    buffer.settings.intensityReductionSetEnable = 1;
    buffer.settings.intensityIncreaseSetEnable = 1;

    // Always enable LED setting change
    buffer.settings.statusLEDSetEnable = 1;

    // Rumble intensity will be ignored. (would require rumbleEmulation)
    buffer.settings.rumbleIntensity = 0;

    // Adaptive trigger data
    memcpy(&buffer.settings.adaptiveTriggerData, &this->adaptiveTriggerData, sizeof(SenseAdaptiveTriggerCommand_t));

    buffer.settings.ledEnable = g_StatusLED ? 0x01 : 0x00;

    // 0x7 is technically 12.5% intensity, but we want to match driver behavior and treat it as off.
    if (buffer.settings.hapticsIntensityReduction != 0x7) {
      buffer.packetNum = this->hapticPacketIncrement++;

      auto& pcmData = this->pcmData;

      // Copy the PCM data to the buffer. We need to make sure we don't go out of bounds.
      size_t bytesToCopy = std::min(pcmData.size() - this->samplesRead, static_cast<size_t>(32));
      if (bytesToCopy != 0)
      {
        memcpy_s(&buffer.hapticPCM, sizeof(buffer.hapticPCM), pcmData.data() + this->samplesRead, bytesToCopy);
        this->samplesRead += bytesToCopy;
      }

      // We basically want to make a thud
      if (this->phaseJump)
      {
        this->phaseJump = false;
        hapticPosition = hapticPosition > k_unSenseHalfSamplePosition ? 0 : k_unSenseHalfSamplePosition;

        bool isStartingAtUp = hapticPosition == 0;

        int8_t amp = static_cast<int8_t>(std::min(this->hapticAmp, static_cast<uint32_t>(k_unSenseMaxHapticAmplitude)));

        for (int i = 0; i < 6; i++)
          buffer.hapticPCM[i] = ClampedAdd(buffer.hapticPCM[i], isStartingAtUp ? amp : -amp);

        for (int i = 6; i < 12; i++)
          buffer.hapticPCM[i] = ClampedAdd(buffer.hapticPCM[i], isStartingAtUp ? -amp : amp);

        // Back up a bit that so the actuator can move down in 6 samples, then all the way up.
        hapticPosition -= 6;
      }

      // Calculate the haptic overdrive based on frequency. We want overdrive to range from 25.0 to 1.0.
      // Basically, this makes a square wave from the cosine wave. Lower frequencies will have a higher overdrive.
      // We also overdrive for frequencies above 500 Hz.

      double overdrive = 25.0;

      // Make sure we don't divide by zero
      if (this->hapticFreq != 0.0)
      {
        overdrive = Clamp(1000.0 / this->hapticFreq, 10.0 + 1.0, 35.0) - 10.0 + (this->hapticFreq - 500.0);
      }

      // In addition to copying the PCM data, we also want to add the generated haptic data to the buffer.
      for (int i = 0; i < sizeof(buffer.hapticPCM); i++)
      {
        if (this->hapticSamplesLeft != 0)
        {
          buffer.hapticPCM[i] = ClampedAdd(buffer.hapticPCM[i], CosineToByte(hapticPosition, k_unSenseMaxSamplePosition, this->hapticAmp, overdrive));
          hapticPosition = (hapticPosition + static_cast<int32_t>(this->hapticFreq * k_unSenseSubsamples)) % k_unSenseMaxSamplePosition;

          this->hapticSamplesLeft -= 1;
        }
      }
    }

    buffer.settings.timeStampMicrosecondsLastSend = static_cast<uint32_t>(GetHostTimestamp());

    *crc = CalculateSenseCRC32(&buffer, sizeof(buffer) - sizeof(buffer.crc));
  }

  long pendingOperationCount = asyncWriter.GetPendingOperationCount();
  if (pendingOperationCount > 2)
  {
    Util::DriverLog("Failed to send. Too many pending operations.\r\n");
    return;
  }

  auto timeoutLambda = [this]() {
    Util::DriverLog("Send operation timed out. Closing device.\r\n");

    CloseHandle(this->handle);

    this->SetHandle(NULL, -1);
  };

  if (!asyncWriter.Write(this->handle, &buffer, 78, 5000, timeoutLambda))
  {
    CloseHandle(this->handle);

    this->SetHandle(NULL, -1);

    Util::DriverLog("Failed to send. Closing device. Last error: {:#x}\r\n", GetLastError());
    return;
  }
};

void SenseThread()
{
  // Set thread importance
  SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);

  auto& leftController = SenseController::GetLeftController();
  auto& rightController = SenseController::GetRightController();

  LARGE_INTEGER frequency;
  QueryPerformanceFrequency(&frequency);

  // Duration we want to run every iteration (32/3000 or 0.010666 seconds)
  LONGLONG duration = static_cast<LONGLONG>((32.0 / 3000.0) * frequency.QuadPart);

  LARGE_INTEGER start;
  QueryPerformanceCounter(&start);

  while (hapticsThread)
  {
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);

    leftController.SendToDevice();
    rightController.SendToDevice();

    if (SenseController::g_ShouldResetLEDTrackingInTicks > 0)
    {
      SenseController::g_ShouldResetLEDTrackingInTicks--;

      static HmdDriverLoader* pHmdDriverLoader = HmdDriverLoader::Instance();
      static char* CaesarManager__resetTrackingFlag = reinterpret_cast<char*>(pHmdDriverLoader->GetBaseAddress() + 0x35b9f5);
      *CaesarManager__resetTrackingFlag = 1;
    }

    LONGLONG elapsed = now.QuadPart - start.QuadPart;

    // Wait out the duration
    while (elapsed < duration)
    {
      // Sleep to not eat up CPU cycles.
      timeBeginPeriod(1); // Set system timer resolution to 1 ms  
      SleepEx(1, TRUE); // Sleep for 1ms, also be alertable for the AsyncFileWriter, which uses APCs.
      timeEndPeriod(1); // Restore system timer resolution

      QueryPerformanceCounter(&now);
      elapsed = now.QuadPart - start.QuadPart;
    }

    // Move to next expected start time
    start.QuadPart += duration;
  }
}

static void PollNextEvent(vr::VREvent_t* pEvent)
{
  static DriverHostProxy* pDriverHostProxy = DriverHostProxy::Instance();

  switch (pEvent->eventType)
  {
  case vr::EVREventType::VREvent_PropertyChanged:
  {
    vr::VREvent_Property_t propertyEvent = *reinterpret_cast<vr::VREvent_Property_t*>(&pEvent->data);

    if (propertyEvent.prop == vr::ETrackedDeviceProperty::Prop_DisplayFrequency_Float)
    {
      SenseController::g_ShouldResetLEDTrackingInTicks = 150;
    }

    break;
  }
  case vr::EVREventType::VREvent_TrackedDeviceUserInteractionEnded:
  {
    if (pEvent->trackedDeviceIndex == vr::k_unTrackedDeviceIndex_Hmd)
    {
      // Turn the controller status LED since the user isn't wearing the headset.
      // The user should be able to know if their controller is on.
      SenseController::g_StatusLED = true;
    }
    break;
  }
  case vr::EVREventType::VREvent_TrackedDeviceUserInteractionStarted:
  {
    if (pEvent->trackedDeviceIndex == vr::k_unTrackedDeviceIndex_Hmd)
    {
      // Conserve power and turn off the status LED.
      // The user is wearing the headset, and they likely don't need to see it.
      SenseController::g_StatusLED = false;
    }
    break;
  }
  case vr::EVREventType::VREvent_Input_HapticVibration:
  {
    vr::VREvent_HapticVibration_t hapticEvent = pEvent->data.hapticVibration;

    // hapticEvent.containerHandle
    DeviceType deviceType = pDriverHostProxy->GetDeviceType(hapticEvent.containerHandle);
    vr::ETrackedControllerRole role;
    if (deviceType == DeviceType::SenseControllerLeft) {
      role = vr::TrackedControllerRole_LeftHand;
    }
    else if (deviceType == DeviceType::SenseControllerRight) {
      role = vr::TrackedControllerRole_RightHand;
    }
    else {
      break;
    }

    float senseHapticFreq = hapticEvent.fFrequency;
    uint8_t senseHapticAmp = 0;
    uint32_t senseHapticSamplesLeft = 0;

    // Phase jump to make a "thud."
    bool phaseJump = hapticEvent.fDurationSeconds == 0.0f;

    if (hapticEvent.fAmplitude != 0.0f)
    {
      senseHapticAmp = static_cast<uint8_t>(sqrtf(hapticEvent.fAmplitude) * k_unSenseMaxHapticAmplitude);
      senseHapticSamplesLeft = phaseJump ? 96 : 16 + static_cast<uint32_t>(hapticEvent.fDurationSeconds * k_unSenseSampleRate);
    }

    SenseController* controller = nullptr;

    if (role == vr::TrackedControllerRole_LeftHand)
    {
      controller = &SenseController::GetLeftController();
    }
    else if (role == vr::TrackedControllerRole_RightHand)
    {
      controller = &SenseController::GetRightController();
    }

    if (controller != nullptr)
    {
      controller->SetGeneratedHaptic(senseHapticFreq, senseHapticAmp, senseHapticSamplesLeft, phaseJump);
    }

    break;
  }
  default:
  {
    break;
  }
  }
}

void psvr2_toolkit::StartSenseThread() {
  hapticsThread = new std::thread(SenseThread);
  hapticsThread.load()->detach();
}

void psvr2_toolkit::StopSenseThread() {
  std::thread* hapticsThreadCopy = hapticsThread;
  hapticsThread = nullptr;

  if (hapticsThreadCopy->joinable())
  {
    hapticsThreadCopy->join();
  }
  delete hapticsThreadCopy;
}

void SenseController::Initialize()
{
  DriverHostProxy::Instance()->SetEventHandler(PollNextEvent);

  StartSenseThread();
}

void SenseController::Destroy()
{
  auto& leftController = SenseController::GetLeftController();
  leftController.SetHandle(NULL, -1);

  auto& rightController = SenseController::GetRightController();
  rightController.SetHandle(NULL, -1);

  StopSenseThread();
}