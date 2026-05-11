#include "common.h"

extern "C" {
  PSVR2TK_EXPORT GazeCalibrationCommand psvr2_toolkit_private_send_gaze_set_command(GazeCalibrationCommand command);
  PSVR2TK_EXPORT GazeCalibrationCommand psvr2_toolkit_private_send_gaze_get_command(GazeCalibrationCommand command);
  PSVR2TK_EXPORT void psvr2_toolkit_private_set_usb_connection_state(bool connected);
}