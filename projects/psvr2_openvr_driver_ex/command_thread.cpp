#include <chrono>

#include "common.h"
#include "driver_interface/caesar_manager.h"
#include "custom_share_manager.h"

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
      DriverCommand* command = CustomShareManager::getSingleton()->popCommand();
      if (command) {
        if (command->type == DriverCommandType::GazeCalibrationSet) {
          CaesarManager* caesarManager = CaesarManager::GetInstance();
          if (caesarManager && caesarManager->imuStatusThread) {
            caesarManager->imuStatusThread->ControlCommand(
              true,
              0xd,
              &command->gazeCalibration.payload,
              sizeof(GazeCalibrationPacket),
              0,
              0,
              reinterpret_cast<uint16_t&>(command->gazeCalibration.reportMode)
            );
          }
        }
        else if (command->type == DriverCommandType::GazeCalibrationGet) {
          CaesarManager* caesarManager = CaesarManager::GetInstance();
          if (caesarManager && caesarManager->imuStatusThread) {
            caesarManager->imuStatusThread->ControlCommand(
              false,
              0x8d,
              &command->gazeCalibration.payload,
              sizeof(GazeCalibrationPacket),
              0,
              0,
              reinterpret_cast<uint16_t&>(command->gazeCalibration.status)
            );
          }
        }

        CustomShareManager::getSingleton()->fulfillCommand(command);
      }
      else {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
  }
}