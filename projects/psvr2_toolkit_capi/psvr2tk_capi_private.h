#include "common.h"

extern "C" {
  __declspec(dllexport) GazeCalibrationCommand psvr2_toolkit_private_send_gaze_set_command(GazeCalibrationCommand command);
  __declspec(dllexport) GazeCalibrationCommand psvr2_toolkit_private_send_gaze_get_command(GazeCalibrationCommand command);
  __declspec(dllexport) void psvr2_toolkit_private_set_usb_connection_state(bool connected);
}