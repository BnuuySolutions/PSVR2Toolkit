#pragma once

#include "common.h"

#define PSVR2TK_CAPI_FUNCTIONS(X) \
  X(int, psvr2_toolkit_init, (), ()) \
  X(void, psvr2_toolkit_deinit, (), ()) \
  X(bool, psvr2_toolkit_gaze_status, (hmd2_gaze_status_t* pGazeStatus, uint32_t timeoutMs), (pGazeStatus, timeoutMs)) \
  X(bool, psvr2_toolkit_gaze_image, (unsigned char* pGazeImage, uint32_t timeoutMs), (pGazeImage, timeoutMs)) \
  X(void, psvr2_toolkit_write_pcm, (VRControllerType controllerType, const unsigned char* pcm), (controllerType, pcm)) \
  X(void, psvr2_toolkit_wait_for_pcm, (), ()) \
  X(void, psvr2_toolkit_set_trigger_effect, (VRControllerType controllerType, const ScePadTriggerEffectCommand& command), (controllerType, command)) \
  X(void, psvr2_toolkit_set_hmd_rumble, (uint8_t rumbleHz), (rumbleHz))

#ifdef __cplusplus
extern "C" {
#endif

#define PSVR2TK_DECLARE_FUNC(ret, name, args_decl, args_pass) PSVR2TK_EXPORT ret name args_decl;
PSVR2TK_CAPI_FUNCTIONS(PSVR2TK_DECLARE_FUNC)
#undef PSVR2TK_DECLARE_FUNC

#ifdef __cplusplus
}
#endif