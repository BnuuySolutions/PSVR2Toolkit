#include "custom_share_manager.h"

extern "C" {

  __declspec(dllexport) void CustomShare_Initialize() {
    CustomShareManager::createSingleton();
  }

  __declspec(dllexport) void CustomShare_GetGazeImage(unsigned char* pGazeImage) {
    CustomShareManager::getSingleton()->getGazeImage(pGazeImage);
  }

}