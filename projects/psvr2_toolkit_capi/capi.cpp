#include "custom_share_manager.h"
#include <cstring>

extern "C" {

  __declspec(dllexport) void CAPI_Initialize() {
    CustomShareManager::createSingleton();
  }

  __declspec(dllexport) void CAPI_GetGazeStatus(unsigned char* pGazeStatus) {
    CustomShareManager::getSingleton()->getGazeStatus(pGazeStatus);
  }

  __declspec(dllexport) void CAPI_GetGazeImage(unsigned char* pGazeImage) {
    unsigned char* ptr = nullptr;
    CustomShareManager::getSingleton()->getGazeImageBuffer(&ptr);
    if (ptr) {
      // TODO: size shouldn't be 0x200100?
      // Also thinking of just making this API return a pointer instead to avoid a copy.
      std::memcpy(pGazeImage, ptr, 0x200100);
    }
  }

}