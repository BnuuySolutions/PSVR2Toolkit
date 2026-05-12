#pragma once

#include "common.h"

#define PSVR2TK_CAPI_PRIVATE_FUNCTIONS(X) \
  X(GazeCalibrationCommand, psvr2_toolkit_private_send_gaze_set_command, (GazeCalibrationCommand command), (command)) \
  X(GazeCalibrationCommand, psvr2_toolkit_private_send_gaze_get_command, (GazeCalibrationCommand command), (command)) \
  X(void, psvr2_toolkit_private_set_usb_connection_state, (bool connected), (connected))

#ifdef __cplusplus
extern "C" {
#endif

#define PSVR2TK_DECLARE_PRIVATE_FUNC(ret, name, args_decl, args_pass) PSVR2TK_EXPORT ret name args_decl;
PSVR2TK_CAPI_PRIVATE_FUNCTIONS(PSVR2TK_DECLARE_PRIVATE_FUNC)
#undef PSVR2TK_DECLARE_PRIVATE_FUNC

#ifdef __cplusplus
}
#endif