#include "custom_share_manager.h"

extern "C" {

  __declspec(dllexport) void CAPI_Initialize() {
    CustomShareManager::createSingleton();
  }

  __declspec(dllexport) void CAPI_GetGazeStatus(unsigned char* pGazeStatus) {
    CustomShareManager::getSingleton()->getGazeStatus(pGazeStatus);
  }

  __declspec(dllexport) void CAPI_GetGazeImage(unsigned char* pGazeImage) {
    CustomShareManager::getSingleton()->getGazeImage(pGazeImage);
  }

}