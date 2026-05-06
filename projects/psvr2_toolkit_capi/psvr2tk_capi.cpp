#include <cstring>

#include "psvr2tk_capi.h"
#include "common.h"
#include "psvr2tk_capi_private.h"
#include "custom_share_manager.h"

extern "C" {

  static int g_slot = -1;

  int psvr2_toolkit_init() {
    CustomShareManager::createSingleton();
    g_slot = CustomShareManager::getSingleton()->claimSlot();
    return g_slot >= 0 ? 0 : -1;
  }

  void psvr2_toolkit_deinit() {
    if (g_slot >= 0) {
      CustomShareManager::getSingleton()->releaseSlot(g_slot);
      g_slot = -1;
    }
  }

  void psvr2_toolkit_gaze_status(hmd2_gaze_status_t* pGazeStatus) {
    CustomShareManager::getSingleton()->getGazeStatus(pGazeStatus);
  }

  void psvr2_toolkit_gaze_image(unsigned char* pGazeImage) {
    unsigned char* ptr = nullptr;
    CustomShareManager::getSingleton()->getGazeImageBuffer(&ptr);
    if (ptr) {
      // TODO: size shouldn't be 0x200100?
      // Also thinking of just making this API return a pointer instead to avoid a copy.
      std::memcpy(pGazeImage, ptr, 0x200100);
    }
  }

  void psvr2_toolkit_write_pcm(VRControllerType controllerType, const unsigned char* pcm) {
    if (g_slot >= 0) CustomShareManager::getSingleton()->writePcm(g_slot, controllerType, pcm);
  }

  void psvr2_toolkit_wait_for_pcm() {
    if (g_slot >= 0) CustomShareManager::getSingleton()->waitForPcmUpdate(g_slot);
  }

  void psvr2_toolkit_set_trigger_effect(VRControllerType controllerType, const ScePadTriggerEffectCommand& command) {
    TriggerEffectCommandPayload payload;
    payload.controllerType = controllerType;
    payload.command = command;
    if (g_slot >= 0) CustomShareManager::getSingleton()->pushTriggerEffect(g_slot, payload);
  }

  GazeCalibrationCommand psvr2_toolkit_private_send_gaze_set_command(GazeCalibrationCommand command) {
    DriverCommand drvCmd = {};
    drvCmd.type = DriverCommandType::GazeCalibrationSet;
    drvCmd.gazeCalibration = command;
    CustomShareManager::getSingleton()->submitCommand(g_slot, drvCmd);
    return drvCmd.gazeCalibration;
  }

  GazeCalibrationCommand psvr2_toolkit_private_send_gaze_get_command(GazeCalibrationCommand command) {
    DriverCommand drvCmd = {};
    drvCmd.type = DriverCommandType::GazeCalibrationGet;
    drvCmd.gazeCalibration = command;
    CustomShareManager::getSingleton()->submitCommand(g_slot, drvCmd);
    return drvCmd.gazeCalibration;
  }
}