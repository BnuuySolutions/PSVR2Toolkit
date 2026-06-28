#include "common.h"
#include "driver_interface/caesar_manager.h"
#include "custom_share_manager.h"
#include "caesar_usb_thread.h"

#include "command_thread.h"

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
    while (m_running) {
      DriverCommand* command = CustomShareManager::getSingleton()->popCommand(10);
      CaesarManager* caesarManager = CaesarManager::GetInstance();

      if (command) {
        if (caesarManager && caesarManager->imuStatusThread) {
          switch (command->type) {
            case DriverCommandType::GazeCalibrationSet:
            {
              caesarManager->imuStatusThread->ControlCommand(
                true,
                0x0D,
                &command->gazeCalibration.payload,
                sizeof(GazeCalibrationPacket),
                0,
                0,
                reinterpret_cast<uint16_t&>(command->gazeCalibration.reportMode)
              );

              break;
            }
            case DriverCommandType::GazeCalibrationGet:
            {
              caesarManager->imuStatusThread->ControlCommand(
                false,
                0x8D,
                &command->gazeCalibration.payload,
                sizeof(GazeCalibrationPacket),
                0,
                0,
                reinterpret_cast<uint16_t&>(command->gazeCalibration.status)
              );
              
              break;
            }
            case DriverCommandType::HeadsetRumbleSet:
            {
              caesarManager->imuStatusThread->ControlCommand(
                true,
                0x08,
                &command->headsetRumble.rumbleHz,
                1,
                0,
                0,
                1);
              
              break;
            }
            case DriverCommandType::UsbConnectionStateSet:
            {
              CaesarUsbThread::SetUsbConnectionState(command->usbConnection.isConnected);
              
              break;
            }
          }
        }

        CustomShareManager::getSingleton()->fulfillCommand(command);
      }
    }
  }
}