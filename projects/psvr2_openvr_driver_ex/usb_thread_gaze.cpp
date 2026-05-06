// #include "common/hmd2_gaze.h" TODO: sort out header re-location
#include "driver_hooks/hmd_device_hooks.h"
#include "driver_interface/caesar_manager.h"
#include "custom_share_manager.h"
#include "usb_thread_gaze.h"
#include "util.h"


#define GAZE_MAGIC_0 0x47
#define GAZE_MAGIC_1_CAL 0x43
#define GAZE_MAGIC_1_RAW 0x52
#define GAZE_MAGIC_1_STATE 0x53

using namespace psvr2_toolkit;

CaesarUsbThreadGaze *CaesarUsbThreadGaze::m_pInstance = nullptr;

uint8_t CaesarUsbThreadGaze::GetInterface() {
  return 5;
}

uint8_t CaesarUsbThreadGaze::GetEndpoint() {
  return 0x85;
}

void CaesarUsbThreadGaze::OnConnected() {
  // For some reason this doesn't really stick.
  CaesarUsbThread::ControlCommandHook(this, true, 12, nullptr, 0, 0, 0, 1);

  //char data[8] = { 1, 0, 0, 0, 0x05, 0, 0, 0 };
  //CaesarUsbThread::ControlCommandHook(this, true, 0xb, data, 8, 0, 0, 1);
}

int CaesarUsbThreadGaze::PollAndProcess() {
  static hmd2_gaze_status_t state;
  int result = CaesarUsbThread::ReadPipeHook(this, 0x85, reinterpret_cast<char*>(&state), sizeof(state));

  if (result == 0) {
    // If we timed out, we should try sending the gaze enable again.
    // Entering and exiting passthrough, DP signal changes, and probably some other stuff seems to stop gaze.
    CaesarUsbThread::ControlCommandHook(this, true, 12, nullptr, 0, 0, 0, 1);
    return 0;
  }

  if (result < 0) {
    return -1;
  }
  
  if (state.magic[0] == GAZE_MAGIC_0 && state.magic[1] == GAZE_MAGIC_1_STATE) {
    HmdDeviceHooks::UpdateGaze(&state, sizeof(hmd2_gaze_status_t));
    CustomShareManager* pShareManager = CustomShareManager::getSingleton();
    pShareManager->setGazeStatus(&state);
  }

  return 0;
}
