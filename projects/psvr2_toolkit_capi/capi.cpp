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

  __declspec(dllexport) int CAPI_ClaimPcmSlot() {
    return CustomShareManager::getSingleton()->claimPcmSlot();
  }

  __declspec(dllexport) void CAPI_ReleasePcmSlot(int slot) {
    CustomShareManager::getSingleton()->releasePcmSlot(slot);
  }

  __declspec(dllexport) void CAPI_WritePcm(int slot, const unsigned char* pcmLeft, const unsigned char* pcmRight) {
    CustomShareManager::getSingleton()->writePcm(slot, pcmLeft, pcmRight);
  }

  __declspec(dllexport) void CAPI_WaitForPcmUpdate(int slot) {
    CustomShareManager::getSingleton()->waitForPcmUpdate(slot);
  }

}