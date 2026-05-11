#include "common.h"

extern "C" {
  PSVR2TK_EXPORT int psvr2_toolkit_init();
  PSVR2TK_EXPORT void psvr2_toolkit_deinit();
  PSVR2TK_EXPORT bool psvr2_toolkit_gaze_status(hmd2_gaze_status_t* pGazeStatus, uint32_t timeoutMs);
  PSVR2TK_EXPORT bool psvr2_toolkit_gaze_image(unsigned char* pGazeImage, uint32_t timeoutMs);
  PSVR2TK_EXPORT void psvr2_toolkit_write_pcm(VRControllerType controllerType, const unsigned char* pcm);
  PSVR2TK_EXPORT void psvr2_toolkit_wait_for_pcm();
  PSVR2TK_EXPORT void psvr2_toolkit_set_trigger_effect(VRControllerType controllerType, const ScePadTriggerEffectCommand& command);
  PSVR2TK_EXPORT void psvr2_toolkit_set_hmd_rumble(uint8_t rumbleHz);
}