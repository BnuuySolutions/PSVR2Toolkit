#include "common.h"
#include "hmd2_gaze.h"
#include "pad_trigger_effect.h"
#include <stdbool.h>

extern "C" {
  __declspec(dllexport) int psvr2_toolkit_init();
  __declspec(dllexport) void psvr2_toolkit_deinit();
  __declspec(dllexport) bool psvr2_toolkit_gaze_status(hmd2_gaze_status_t* pGazeStatus, uint32_t timeoutMs);
  __declspec(dllexport) bool psvr2_toolkit_gaze_image(unsigned char* pGazeImage, uint32_t timeoutMs);
  __declspec(dllexport) void psvr2_toolkit_write_pcm(VRControllerType controllerType, const unsigned char* pcm);
  __declspec(dllexport) void psvr2_toolkit_wait_for_pcm();
  __declspec(dllexport) void psvr2_toolkit_set_trigger_effect(VRControllerType controllerType, const ScePadTriggerEffectCommand& command);
  __declspec(dllexport) void psvr2_toolkit_set_hmd_rumble(uint8_t rumbleHz);
}