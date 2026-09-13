#include "common.h"
#include "driver_interface/caesar_manager.h"
#include "custom_share_manager.h"
#include "caesar_usb_thread.h"
#include "trigger_effect_manager.h"

#include "command_thread.h"
#include "util.h"

namespace psvr2_toolkit {
std::atomic<bool> CommandThread::m_running{false};
std::thread CommandThread::m_thread;

void CommandThread::Initialize() {
  if (!m_running) {
    m_running = true;
    m_thread = std::thread(&CommandThread::ThreadLoop);
  }
}

void CommandThread::Stop() {
  if (m_running) {
    m_running = false;
    if (m_thread.joinable()) {
      m_thread.join();
    }
  }
}

void CommandThread::ThreadLoop() {
  CustomShareManager *customShareManager = CustomShareManager::getSingleton();

  // Losing this race must not take the thread down with it. The mutex only backs the "driver active"
  // flag that clients poll, whereas TriggerEffectManager::Update below drives controller trigger
  // effects every ~10 ms and has to keep running either way. The original code span here until it won
  // and never reported failure, so bailing out on a 5 second timeout would have been a regression.
  const bool holdsDriverMutex = customShareManager->claimDriverMutex();
  if (!holdsDriverMutex) {
    Util::DriverLog("[CommandThread] Could not claim the driver-active mutex within the timeout. Clients will not "
                    "see the driver as active, but trigger effects will still run.");
  }

  while (m_running) {
    DriverCommand *command = customShareManager->popCommand(10);
    CaesarManager *caesarManager = CaesarManager::getSingleton();

    // This should run about every 10ms since popCommand is set to timeout after 10ms.
    TriggerEffectManager::Instance()->Update();

    if (command) {
      if (caesarManager && caesarManager->imuStatusThread) {
        switch (command->type) {
        case DriverCommandType::GazeCalibrationSet: {
          caesarManager->imuStatusThread->ControlCommand(true, 0x0D, &command->gazeCalibration.payload, sizeof(GazeCalibrationPacket), 0, 0,
                                                         reinterpret_cast<uint16_t &>(command->gazeCalibration.reportMode));

          break;
        }
        case DriverCommandType::GazeCalibrationGet: {
          caesarManager->imuStatusThread->ControlCommand(false, 0x8D, &command->gazeCalibration.payload, sizeof(GazeCalibrationPacket), 0, 0,
                                                         reinterpret_cast<uint16_t &>(command->gazeCalibration.status));

          break;
        }
        case DriverCommandType::HeadsetRumbleSet: {
          caesarManager->imuStatusThread->ControlCommand(true, 0x08, &command->headsetRumble.rumbleHz, 1, 0, 0, 1);

          break;
        }
        case DriverCommandType::UsbConnectionStateSet: {
          CaesarUsbThread::SetUsbConnectionState(command->usbConnection.isConnected);

          break;
        }
        case DriverCommandType::TriggerEffectSet: {
          TriggerEffectManager::Instance()->SetSlotEffect(command->triggerEffect.slot, command->triggerEffect.payload);

          break;
        }
        }
      }

      customShareManager->fulfillCommand(command);
    }
  }

  // Only release what we actually took: unlocking a mutex this thread never owned would hand it away
  // from whichever process does own it.
  if (holdsDriverMutex) {
    customShareManager->releaseDriverMutex();
  }
}
} // namespace psvr2_toolkit