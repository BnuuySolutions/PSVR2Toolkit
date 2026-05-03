#include "common.h"
#include "hmd2_gaze.h"

extern "C" {
  __declspec(dllexport) int CAPI_Initialize();
  __declspec(dllexport) void CAPI_Deinitialize();
  __declspec(dllexport) void CAPI_GetGazeStatus(hmd2_gaze_status_t* pGazeStatus);
  __declspec(dllexport) void CAPI_GetGazeImage(unsigned char* pGazeImage);
  __declspec(dllexport) void CAPI_WritePcm(VRControllerType controllerType, const unsigned char* pcm);
  __declspec(dllexport) void CAPI_WaitForPcmUpdate();
  __declspec(dllexport) void CAPI_SendTriggerEffect(const TriggerEffectCommandPayload* payload);
}