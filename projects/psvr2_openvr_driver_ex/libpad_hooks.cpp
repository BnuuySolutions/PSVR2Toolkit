#include "libpad_hooks.h"

#include "hmd_driver_loader.h"
#include "hook_lib.h"
#include "sense_controller.h"
#include "usb_thread_gaze.h"
#include "usb_thread_hooks.h"
#include "util.h"
#include "vr_settings.h"

#include <cstdint>

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

  struct HidDeviceDescriptor
  {
    uint32_t unknown1[2];
    int32_t padHandle;
    uint32_t unknown2;
    HANDLE controllerFileHandle;
  };

  enum class CommandType : uint8_t {
    SET_SYNC_PHASE = 1,
    SET_LEDS_IMMEDIATE = 2,
    ADJUST_FRAME_CYCLE = 3,
    ADJUST_BASE_TIME = 4,
    ADJUST_TIME_AND_CYCLE = 5,
    SYSTEM_CONTROL = 7
  };

#pragma pack(push, 1)
  struct SetSyncPhasePayload {
    uint8_t  phase;
    uint8_t  period;
    uint8_t  leds[4];
    union {
      uint32_t frameCycle;
      int32_t offset;
    };
  };

  struct SetLedsPayload {
    uint8_t leds[4];
  };

  struct AdjustFrameCyclePayload {
    float adjustmentFactor;
  };

  struct AdjustBaseTimePayload {
    int32_t offset;
  };

  struct AdjustTimeAndCyclePayload {
    float    adjustmentFactor;
    int32_t offset;
  };

  struct SystemControlPayload {
    uint8_t subCommand;
    uint8_t subCommandPayload;
  };

  struct LedCommand {
    CommandType type;

    union Payload {
      SetSyncPhasePayload       syncPhase;
      SetLedsPayload            setLeds;
      AdjustFrameCyclePayload   adjustCycle;
      AdjustBaseTimePayload     adjustTime;
      AdjustTimeAndCyclePayload adjustTimeAndCycle;
      SystemControlPayload      sysControl;
    } payload;
  };
#pragma pack(pop)

  const int32_t k_libpadDeviceTypeSenseLeft = 3;
  const int32_t k_libpadDeviceTypeSenseRight = 4;

  const int32_t k_driverDeviceTypeHmd = 0;
  const int32_t k_driverDeviceTypeSenseLeft = 2;
  const int32_t k_driverDeviceTypeSenseRight = 1;

  const int32_t k_prescanPhasePeriod = 32;
  const int32_t k_broadPhasePeriod = 32;
  const int32_t k_bgPhasePeriod = 20;
  const int32_t k_stablePhasePeriod = 9;

  uintptr_t packetRecievedReturnAddress = 0;

  enum class CalibrationState {
    Idle = -1,
    Start,
    FindInitialOnPoint,
    BinarySearchLeftEdge,
    BinarySearchRightEdge,
    ConfirmLeftEdgeOuter,
    ConfirmLeftEdgeInner,
    ConfirmRightEdgeOuter,
    ConfirmRightEdgeInner,
    Finalize
  };

  struct ControllerContext {
    CalibrationState state = CalibrationState::Idle;
    int32_t thresholdLedCount = 0;
    uint64_t syncStartTime = 0;
    uint64_t lastSync = 0;
    uint64_t lastSyncFrame = 0;
    int32_t searchLowerBound = 0;
    int32_t searchUpperBound = 16666;
    int32_t leftEdge = 0;
    int32_t rightEdge = 0;

    int32_t tunedCycle = 16683350; // 59.94hz. When we hit STABLE, we'll replace this.
  };
  static ControllerContext controllerCtx[2];
  std::atomic<int32_t> g_controllerLedCount[2] = {0, 0};
  std::atomic<uint64_t> g_opticalFrameIndex[2] = {0, 0};

  void (*OpticalProcessor__process)(void *pContext, void *pOpticalData) = nullptr;
  void OpticalProcessor__processHook(void *pContext, void *pOpticalData) {
    uint32_t controllerIdx = *reinterpret_cast<uint32_t *>(reinterpret_cast<uint8_t *>(pContext) + 8);
    
    if (controllerIdx < 2) {
      uint8_t *pControllerData = reinterpret_cast<uint8_t *>(pOpticalData) + (controllerIdx * 0x5A44);
      int32_t currentLedCount = 0;
      
      for (int cam = 0; cam < 4; cam++) {
        uint8_t *pCamData = pControllerData + (cam * 0x1688);
        for (int ledId = 0; ledId < 17; ledId++) {
          int16_t blobIndex = *reinterpret_cast<int16_t *>(pCamData + 0x1438 + (ledId * 2));
          if (blobIndex != -1) {
            uint8_t *pBlobData = pCamData + 0x40 + (blobIndex * 0x14);
            int16_t isMatched = *reinterpret_cast<int16_t *>(pBlobData + 8);
            if (isMatched == 1) {
              currentLedCount++;
            }
          }
        }
      }
      g_controllerLedCount[controllerIdx] = currentLedCount;
      g_opticalFrameIndex[controllerIdx]++;
    }
    OpticalProcessor__process(pContext, pOpticalData);
  }

  uint32_t libpad_hostToDeviceHook(LibpadTimeSync *timeSync, uint32_t host, uint32_t *outDevice) {
    SenseController &senseController = SenseController::GetControllerByIsLeft(timeSync->isLeft);

    uint64_t calculatedTimestamp = static_cast<uint64_t>(host)
      + static_cast<uint64_t>(senseController.GetTimestampOffset());

    *outDevice = static_cast<uint32_t>(calculatedTimestamp % k_unDeviceTimestampModulus);

    return 0;
  }

  uint32_t libpad_deviceToHostHook(LibpadTimeSync *timeSync, uint32_t device, ProcessedControllerState *outHost) {
    SenseController &senseController = SenseController::GetControllerByIsLeft(timeSync->isLeft);

    uint64_t currentTime = GetHostTimestamp();

    // We need a specific caller that actually passes ProcessedControllerState* into outHost.
    // This caller should be from when a controller packet is received.
    if (reinterpret_cast<uintptr_t>(_ReturnAddress()) == packetRecievedReturnAddress)
    {
      uint32_t deviceTimestamp = outHost->deviceTimestamp / k_unSenseUnitsPerMicrosecond;

      int32_t clockOffset = GetWraparoundDifference(deviceTimestamp, static_cast<uint32_t>(currentTime));

      senseController.AddTimestampOffsetSample(static_cast<double>(clockOffset));
    }

    // Translate the device timestamp back to the host's 64-bit time domain.
    uint32_t hostTime = (static_cast<uint32_t>(device)
      - static_cast<uint32_t>(senseController.GetTimestampOffset())
      + k_unDeviceTimestampModulus) % k_unDeviceTimestampModulus;

    int32_t difference = GetWraparoundDifference(hostTime, static_cast<uint32_t>(currentTime));

    outHost->hostReceiveTime = currentTime + difference;

    return 0;
  }

  void (*libpad_SetSyncLedCommand)(LibpadTimeSync *timeSync, LibpadLedSync *ledSync, LedCommand *ledCommand, uint8_t commandSize, bool isLeft) = nullptr;
  void libpad_SetSyncLedCommandHook(LibpadTimeSync *timeSync, LibpadLedSync *ledSync, LedCommand *ledCommand, uint8_t commandSize, bool isLeft) {
    static std::mutex ledCommandMutex;
    std::scoped_lock lock(ledCommandMutex);

    int32_t controller = isLeft ? 1 : 0;
    if (controllerCtx[controller].state != CalibrationState::Idle)
    {
      // Don't allow the driver to issue LED commands while calibrating.
      return;
    }

    switch (ledCommand->type) {
    case CommandType::SET_SYNC_PHASE:

      // Use lower LED period time for better battery life.
      switch (ledCommand->payload.syncPhase.phase)
      {
      case LedPhase::PRESCAN:
        ledCommand->payload.syncPhase.period = k_prescanPhasePeriod;
        break;
      case LedPhase::BROAD:
        ledCommand->payload.syncPhase.period = k_broadPhasePeriod;
        break;
      case LedPhase::BG:
        ledCommand->payload.syncPhase.period = k_bgPhasePeriod;
        break;
      case LedPhase::STABLE:
        ledCommand->payload.syncPhase.period = k_stablePhasePeriod;

        // Ensure we correctly line up with the center due to the difference in period
        ledCommand->payload.syncPhase.offset = (ledSync->oneSubGridTime * (k_bgPhasePeriod - k_stablePhasePeriod));

        // Store the new tuned frequency to make it so we work less on future led syncs.
        controllerCtx[controller].tunedCycle = ledSync->frameCycle; 

        break;
      }

      if (ledCommand->payload.syncPhase.phase == LedPhase::PRESCAN) {
        ledCommand->payload.syncPhase.frameCycle = controllerCtx[controller].tunedCycle;
        Util::DriverLog("SET_SYNC_PHASE: phase={}, period={}, frameCycle={}, leds=[{},{},{},{}] {}",
          ledCommand->payload.syncPhase.phase,
          ledCommand->payload.syncPhase.period,
          ledCommand->payload.syncPhase.frameCycle,
          ledCommand->payload.syncPhase.leds[0],
          ledCommand->payload.syncPhase.leds[1],
          ledCommand->payload.syncPhase.leds[2],
          ledCommand->payload.syncPhase.leds[3], commandSize);
      }
      else {
        Util::DriverLog("SET_SYNC_PHASE: phase={}, period={}, offset={}, leds=[{},{},{},{}] {}",
          ledCommand->payload.syncPhase.phase,
          ledCommand->payload.syncPhase.period,
          ledCommand->payload.syncPhase.offset,
          ledCommand->payload.syncPhase.leds[0],
          ledCommand->payload.syncPhase.leds[1],
          ledCommand->payload.syncPhase.leds[2],
          ledCommand->payload.syncPhase.leds[3], commandSize);
      }

      break;
    case CommandType::SET_LEDS_IMMEDIATE:
      Util::DriverLog("SET_LEDS_IMMEDIATE: leds=[{},{},{},{}]",
        ledCommand->payload.setLeds.leds[0],
        ledCommand->payload.setLeds.leds[1],
        ledCommand->payload.setLeds.leds[2],
        ledCommand->payload.setLeds.leds[3]);
      break;
    case CommandType::ADJUST_FRAME_CYCLE:
      Util::DriverLog("ADJUST_FRAME_CYCLE: adjustmentFactor={}",
        ledCommand->payload.adjustCycle.adjustmentFactor);
      break;
    case CommandType::ADJUST_BASE_TIME:
      Util::DriverLog("ADJUST_BASE_TIME: offset={}",
        ledCommand->payload.adjustTime.offset);
      break;
    case CommandType::ADJUST_TIME_AND_CYCLE:
      Util::DriverLog("ADJUST_TIME_AND_CYCLE: adjustmentFactor={}, offset={}, ledsync.frameCycle={}",
        ledCommand->payload.adjustTimeAndCycle.adjustmentFactor,
        ledCommand->payload.adjustTimeAndCycle.offset,
        ledSync->frameCycle);
      break;
    case CommandType::SYSTEM_CONTROL:
      Util::DriverLog("SYSTEM_CONTROL: subCommand={}, subCommandPayload={}",
        ledCommand->payload.sysControl.subCommand,
        ledCommand->payload.sysControl.subCommandPayload);
      break;
    default:
      break;
    }

    libpad_SetSyncLedCommand(timeSync, ledSync, ledCommand, commandSize, isLeft);
  }

  void (*libpad_SetSyncLedBaseTime)(LibpadTimeSync *timeSync, LibpadLedSync *ledSync) = nullptr;
  void libpad_SetSyncLedBaseTimeHook(LibpadTimeSync *timeSync, LibpadLedSync *ledSync) {
    static std::mutex ledSyncMutex;
    std::scoped_lock lock(ledSyncMutex);

    int32_t controller = timeSync->isLeft ? 1 : 0;
    char controllerChar = timeSync->isLeft ? 'L' : 'R';
    uint64_t lastTrackedTimestamp;

    SenseController &senseController = SenseController::GetControllerByIsLeft(timeSync->isLeft);

    senseController.SetLibpadSyncs(timeSync, ledSync);

    int32_t latencyOffset = senseController.GetLatencyOffset();
    size_t hasTimeOffset = senseController.GetHasTimestampOffset();
    bool isTracking = senseController.GetTrackingState(lastTrackedTimestamp);

    ControllerContext &ctx = controllerCtx[controller];

    auto resetCalibration = [&]() {
      senseController.SetLatencyOffset(-1);
      ctx.state = CalibrationState::Start;
      ctx.syncStartTime = GetHostTimestamp();
      ctx.lastSyncFrame = g_opticalFrameIndex[controller];
      ctx.searchLowerBound = 0;
      ctx.searchUpperBound = 16666;
      Util::DriverLog("[{}] Latency calibration has been reset.", controllerChar);
    };

    int32_t currentLedCount = g_controllerLedCount[controller];

    // TODO: calibration should be triggered via IPC
    static bool f9Pressed = false;
    if (GetAsyncKeyState(VK_F9) & 0x8000) {
      if (!f9Pressed) {
        controllerCtx[0].state = CalibrationState::Idle;
        controllerCtx[1].state = CalibrationState::Idle;
        SenseController::GetLeftController().SetLatencyOffset(-1);
        SenseController::GetRightController().SetLatencyOffset(-1);
        f9Pressed = true;
      }
    } else {
      f9Pressed = false;
    }

    if (senseController.GetHandle() == NULL) {
      if (ctx.state != CalibrationState::Idle) {
        resetCalibration();
        ctx.state = CalibrationState::Idle;
        Util::DriverLog("[{}] Controller disconnected, stopping latency calibration.", controllerChar);
      }
    }

    if (hasTimeOffset && latencyOffset == -1 && ctx.state == CalibrationState::Idle) {
      ctx.state = CalibrationState::Start;
      senseController.SetLatencyOffset(0);
      ctx.syncStartTime = GetHostTimestamp();
      ctx.lastSyncFrame = g_opticalFrameIndex[controller];
    }

    if (ctx.state != CalibrationState::Idle
      && GetHostTimestamp() - ctx.syncStartTime > 300000) {
      // Force to PRESCAN phase while calibrating.
      ledSync->phase = PRESCAN;
      ledSync->period = k_prescanPhasePeriod;

      int32_t fullWindow = (ledSync->oneSubGridTime * static_cast<int32_t>(ledSync->period)) + ledSync->camExposure;

      if (g_opticalFrameIndex[controller] - ctx.lastSyncFrame >= 4) {
        int32_t newLatencyOffset = 0;

        switch (ctx.state) {
        case CalibrationState::Start:
          ctx.thresholdLedCount = currentLedCount + 3;
          ctx.state = CalibrationState::FindInitialOnPoint;
          break;

        case CalibrationState::FindInitialOnPoint:
        {
          newLatencyOffset = latencyOffset + fullWindow;

          if (currentLedCount > ctx.thresholdLedCount) { // LEDs are on, we found a point
            ctx.searchUpperBound = latencyOffset;
            ctx.searchLowerBound = latencyOffset - fullWindow;
            Util::DriverLog("[{}] Found initial on point32_t at {}. Searching for left edge.", controllerChar, latencyOffset);
            ctx.state = CalibrationState::BinarySearchLeftEdge;

          }
          else { // LEDs are off
            senseController.SetLatencyOffset(newLatencyOffset);
            ctx.searchLowerBound = newLatencyOffset;
            if (ctx.searchUpperBound - ctx.searchLowerBound < fullWindow / 2) {
              Util::DriverLog("[{}] Could not find an initial on point.", controllerChar);
              resetCalibration();
            }
            break;
          }

          [[fallthrough]];
        }

        case CalibrationState::BinarySearchLeftEdge:
        {
          if (currentLedCount > ctx.thresholdLedCount) { // LEDs are on
            ctx.searchUpperBound = latencyOffset;
          }
          else { // LEDs are off
            ctx.searchLowerBound = latencyOffset;
          }

          newLatencyOffset = (ctx.searchLowerBound + ctx.searchUpperBound) / 2;

          if (ctx.searchUpperBound - ctx.searchLowerBound < 25) {
            ctx.leftEdge = newLatencyOffset;
            Util::DriverLog("[{}] Found left edge at {}", controllerChar, ctx.leftEdge);
            ctx.searchLowerBound = ctx.leftEdge + static_cast<int32_t>(fullWindow * 0.9);
            ctx.searchUpperBound = ctx.leftEdge + static_cast<int32_t>(fullWindow * 1.1);
            ctx.state = CalibrationState::BinarySearchRightEdge;
          }
          else {
            senseController.SetLatencyOffset(newLatencyOffset);
            break;
          }

          [[fallthrough]];
        }

        case CalibrationState::BinarySearchRightEdge:
        {
          if (ctx.state == CalibrationState::BinarySearchRightEdge) {
            if (currentLedCount > ctx.thresholdLedCount) { // LEDs are on
              ctx.searchLowerBound = latencyOffset;
            }
            else { // LEDs are off
              ctx.searchUpperBound = latencyOffset;
            }
          }
          else {
            // We just fell through. Use the bounds from the last step.
            ctx.state = CalibrationState::BinarySearchRightEdge;
          }

          newLatencyOffset = (ctx.searchLowerBound + ctx.searchUpperBound) / 2;

          if (ctx.searchUpperBound - ctx.searchLowerBound < 25) {
            ctx.rightEdge = newLatencyOffset;
            Util::DriverLog("[{}] Found right edge at {}", controllerChar, ctx.rightEdge);
            ctx.state = CalibrationState::ConfirmLeftEdgeOuter;
            senseController.SetLatencyOffset(ctx.leftEdge - 50);
          }
          else {
            senseController.SetLatencyOffset(newLatencyOffset);
          }

          break;
        }

        case CalibrationState::ConfirmLeftEdgeOuter:
          if (currentLedCount > ctx.thresholdLedCount) { // Should be off
            Util::DriverLog("[{}] Left edge confirmation failed (outer).", controllerChar);
            resetCalibration();
            break;
          }
          Util::DriverLog("[{}] Left edge confirmation passed (outer).", controllerChar);
          ctx.state = CalibrationState::ConfirmLeftEdgeInner;
          senseController.SetLatencyOffset(ctx.leftEdge + 50);
          break;

        case CalibrationState::ConfirmLeftEdgeInner:
          if (currentLedCount <= ctx.thresholdLedCount) { // Should be on
            Util::DriverLog("[{}] Left edge confirmation failed (inner).", controllerChar);
            resetCalibration();
            break;
          }
          Util::DriverLog("[{}] Left edge confirmation passed (inner).", controllerChar);
          ctx.state = CalibrationState::ConfirmRightEdgeOuter;
          senseController.SetLatencyOffset(ctx.rightEdge + 50);
          break;

        case CalibrationState::ConfirmRightEdgeOuter:
          if (currentLedCount > ctx.thresholdLedCount) { // Should be off
            Util::DriverLog("[{}] Right edge confirmation failed (outer).", controllerChar);
            resetCalibration();
            break;
          }
          Util::DriverLog("[{}] Right edge confirmation passed (outer).", controllerChar);
          senseController.SetLatencyOffset(ctx.rightEdge - 50);
          ctx.state = CalibrationState::ConfirmRightEdgeInner;
          break;

        case CalibrationState::ConfirmRightEdgeInner:
          if (currentLedCount <= ctx.thresholdLedCount) { // Should be on
            Util::DriverLog("[{}] Right edge confirmation failed (inner).", controllerChar);
            resetCalibration();
            break;
          }
          Util::DriverLog("[{}] Right edge confirmation passed (inner).", controllerChar);
          ctx.state = CalibrationState::Finalize;
          [[fallthrough]];

        case CalibrationState::Finalize:
          ctx.state = CalibrationState::Idle;

          int32_t finalOffset = ctx.leftEdge + (ctx.rightEdge - ctx.leftEdge) / 2;

          senseController.SetLatencyOffset(finalOffset);

          Util::DriverLog("[{}] Final latency offset: {} microseconds. LED on time: actual: {} expected: {}", controllerChar, finalOffset, ctx.rightEdge - ctx.leftEdge, fullWindow);

          {
            LibpadTimeSync *controllerTimeSync;
            LibpadLedSync *controllerLedSync;

            senseController.GetLibpadSyncs(controllerTimeSync, controllerLedSync);

            if (controllerTimeSync != nullptr && controllerLedSync != nullptr) {
              LedCommand command = {};
              command.type = CommandType::SET_SYNC_PHASE;
              command.payload.syncPhase.phase = PRESCAN;
              command.payload.syncPhase.period = k_prescanPhasePeriod;
              memset(command.payload.syncPhase.leds, 0xFF, sizeof(command.payload.syncPhase.leds));

              libpad_SetSyncLedCommand(
                controllerTimeSync,
                controllerLedSync,
                &command, sizeof(command.type) + sizeof(command.payload.syncPhase) - sizeof(command.payload.syncPhase.frameCycle),
                senseController.isLeft);

              SenseController::g_ShouldResetLEDTrackingInTicks = 20;
            }
          }
          break;
        }

        if (ctx.state != CalibrationState::Idle) {
          LedCommand command = {};
          command.type = CommandType::SET_SYNC_PHASE;
          command.payload.syncPhase.phase = PRESCAN;
          command.payload.syncPhase.period = k_prescanPhasePeriod;
          memset(command.payload.syncPhase.leds, 0xFF, sizeof(command.payload.syncPhase.leds));

          libpad_SetSyncLedCommand(
            timeSync,
            ledSync,
            &command, sizeof(command.type) + sizeof(command.payload.syncPhase) - sizeof(command.payload.syncPhase.frameCycle),
            timeSync->isLeft);

          Util::DriverLog("[{}] Latency Offset is currently at {} microseconds. Current LED count: {} Threshold: {} Step: {}", controllerChar, senseController.GetLatencyOffset(), currentLedCount, ctx.thresholdLedCount, (int)ctx.state);

          ctx.lastSync = GetHostTimestamp();
          ctx.lastSyncFrame = g_opticalFrameIndex[controller];
        }
      }
    }
    else if (ctx.state != CalibrationState::Idle) {
      // Ensure LEDs are off before we start calibration
      ledSync->phase = LED_ALL_OFF;
      ledSync->seq++;

      // Set threshold LED count.
      // Also add a small margin of error.
      if (ctx.state == CalibrationState::Start) {
        ctx.thresholdLedCount = currentLedCount + 3;
      }
    }
    else if (GetHostTimestamp() - ctx.lastSync < 500000) {
      // If we haven't tracked in the last half of a second, reset the calibration.
      if (!isTracking && static_cast<int64_t>(GetHostTimestamp() - lastTrackedTimestamp) > 500000) {
        Util::DriverLog("[{}] Reset latency calibration due to controller not tracking. {}", controllerChar, static_cast<int64_t>(GetHostTimestamp() - lastTrackedTimestamp));
        resetCalibration();
      }
    }

    // This call uses libpad_hostToDevice, which will factor in the updated latencyOffset.
    libpad_SetSyncLedBaseTime(timeSync, ledSync);
  }

  void (*logDeviceTrackingState)(void *session, int32_t deviceType, uint64_t timestamp,
    void *previousPose, void *previousMeta, void *currentPose, void *currentMeta) = nullptr;
  void logDeviceTrackingStateHook(void *session, int32_t deviceType, uint64_t timestamp,
    void *previousPose, void *previousMeta, void *currentPose, void *currentMeta) {
    char *trackingFlag = reinterpret_cast<char *>(currentMeta) + 0x8;

    int32_t controller = -1;

    if (deviceType == k_driverDeviceTypeSenseLeft) {
      controller = 0;
    }
    else if (deviceType == k_driverDeviceTypeSenseRight) {
      controller = 1;
    }

    if (controller != -1)
    {
      SenseController &senseController = SenseController::GetControllerByIsLeft(controller == 0 ? true : false);
      bool isTracking = (*trackingFlag == 9);

      senseController.SetIsTracking(isTracking, GetHostTimestamp());
    }

    logDeviceTrackingState(session, deviceType, timestamp,
      previousPose, previousMeta, currentPose, currentMeta);
  }

  HidDeviceDescriptor *(*libpad_CreateHidDevice)(HidDeviceDescriptor *device, const wchar_t *name, int32_t deviceType) = nullptr;
  HidDeviceDescriptor *libpad_CreateHidDeviceHook(HidDeviceDescriptor *device, const wchar_t *name, int32_t deviceType) {
    Util::DriverLog("libpad_CreateHidDeviceHook called for device: {}, type: {}", Util::WideStringToUTF8(name), deviceType);

    HidDeviceDescriptor *result = libpad_CreateHidDevice(device, name, deviceType);

    if (deviceType == k_libpadDeviceTypeSenseLeft)
    {
      SenseController::GetLeftController().SetHandle(device->controllerFileHandle, result->padHandle);
    }
    else if (deviceType == k_libpadDeviceTypeSenseRight)
    {
      SenseController::GetRightController().SetHandle(device->controllerFileHandle, result->padHandle);
    }

    return result;
  }

  void (*libpad_Disconnect)(int32_t device) = nullptr;
  void libpad_DisconnectHook(int32_t device) {
    Util::DriverLog("libpad_DisconnectHook called for device handle: {}", device);

    try {
      SenseController &controller = SenseController::GetControllerByPadHandle(device);
      controller.SetHandle(NULL, -1);
    }
    catch (const std::runtime_error &) {
      // No controller with the given pad handle.
    }

    libpad_Disconnect(device);
  }

  int32_t(*libpad_SendOutputReport)(int32_t device, const uint8_t *buffer, uint16_t size) = nullptr;
  int32_t libpad_SendOutputReportHook(int32_t device, const uint8_t *buffer, uint16_t size) {
    if (size != sizeof(SenseControllerPCModePacket_t)) {
      Util::DriverLog("libpad_SendOutputReportHook called with unexpected size: {}", size);
      return libpad_SendOutputReport(device, buffer, size);
    }

    try {
      SenseController &controller = SenseController::GetControllerByPadHandle(device);
      controller.SetTrackingControllerSettings(reinterpret_cast<const SenseControllerPCModePacket_t *>(buffer));
    }
    catch (const std::runtime_error &) {
      // No controller with the given pad handle.
      return 0x8001002b;
    }

    return 0;
  }

  void LibpadHooks::InstallHooks() {
    static HmdDriverLoader *pHmdDriverLoader = HmdDriverLoader::Instance();

    if (VRSettings::GetBool(STEAMVR_SETTINGS_USE_TOOLKIT_SYNC, SETTING_USE_TOOLKIT_SYNC_DEFAULT_VALUE)) {
      Util::DriverLog("Using custom controller/LED sync...");

      packetRecievedReturnAddress = pHmdDriverLoader->GetBaseAddress() + 0x1c75a1;

      // libpad function for uint32_t libpad_hostToDevice(LibpadTimeSync* timeSync, uint32_t host, uint32_t* outDevice) @ 0x1C09E0
      HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x1C09E0),
        reinterpret_cast<void *>(libpad_hostToDeviceHook));

      // libpad function for uint32_t libpad_deviceToHost(LibpadTimeSync* timeSync, uint32_t device, ProcessedControllerState* outHost) @ 0x1C1520
      HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x1C1520),
        reinterpret_cast<void *>(libpad_deviceToHostHook));

      // libpad function for void libpad_SetSyncLedCommand(LibpadTimeSync* timeSync, LibpadLedSync* ledSync, LedCommand* ledCommand, uint8_t commandSize, bool isLeft) @ 0x1C1880
      HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x1C1880),
        reinterpret_cast<void *>(libpad_SetSyncLedCommandHook),
        reinterpret_cast<void **>(&libpad_SetSyncLedCommand));

      // libpad function for void libpad_SetSyncLedBaseTime(LibpadTimeSync* timeSync, LibpadLedSync* ledSync) @ 0x1C27D0
      HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x1C27D0),
        reinterpret_cast<void *>(libpad_SetSyncLedBaseTimeHook),
        reinterpret_cast<void **>(&libpad_SetSyncLedBaseTime));

      // OpticalProcessor::process @ 0x1999F0
      HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x1999F0),
        reinterpret_cast<void *>(OpticalProcessor__processHook),
        reinterpret_cast<void **>(&OpticalProcessor__process));

      // TODO: this should be moved out
      // driver function for void logDeviceTrackingState(void* session, int32_t deviceType, uint64_t timestamp, void* previousPose, void* previousMeta, void* currentPose, void* currentMeta) @ 0x161520
      HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x161520),
        reinterpret_cast<void *>(logDeviceTrackingStateHook),
        reinterpret_cast<void **>(&logDeviceTrackingState));
    }

    // libpad function for int32_t CreateHidDevice(HidDeviceDescriptor* device, wchar_t name, int32_t deviceType) @ 0x1CE210
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x1CE210),
      reinterpret_cast<void *>(libpad_CreateHidDeviceHook),
      reinterpret_cast<void **>(&libpad_CreateHidDevice));

    // libpad function for void libpad_Disconnect(int32_t device) @ 0x1C7E90
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x1C7E90),
      reinterpret_cast<void *>(libpad_DisconnectHook),
      reinterpret_cast<void **>(&libpad_Disconnect));

    if (VRSettings::GetBool(STEAMVR_SETTINGS_USE_ENHANCED_HAPTICS, SETTING_USE_TOOLKIT_SYNC_DEFAULT_VALUE)) {
      // libpad function for int32_t libpad_SendOutputReport(int32_t handle, uchar * buffer, uint16_t size) @ 0x1CBA20
      HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x1CBA20),
        reinterpret_cast<void *>(libpad_SendOutputReportHook),
        reinterpret_cast<void **>(&libpad_SendOutputReport));
    }
  }

} // psvr2_toolkit