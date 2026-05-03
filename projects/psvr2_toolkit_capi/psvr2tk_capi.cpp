#include <cstring>

#include "psvr2tk_capi.h"
#include "custom_share_manager.h"

extern "C" {

  static int g_slot = -1;

  int CAPI_Initialize() {
    CustomShareManager::createSingleton();
    g_slot = CustomShareManager::getSingleton()->claimSlot();
    return g_slot >= 0 ? 0 : -1;
  }

  void CAPI_Deinitialize() {
    if (g_slot >= 0) {
      CustomShareManager::getSingleton()->releaseSlot(g_slot);
      g_slot = -1;
    }
  }

  void CAPI_GetGazeStatus(hmd2_gaze_status_t* pGazeStatus) {
    CustomShareManager::getSingleton()->getGazeStatus(pGazeStatus);
  }

  void CAPI_GetGazeImage(unsigned char* pGazeImage) {
    unsigned char* ptr = nullptr;
    CustomShareManager::getSingleton()->getGazeImageBuffer(&ptr);
    if (ptr) {
      // TODO: size shouldn't be 0x200100?
      // Also thinking of just making this API return a pointer instead to avoid a copy.
      std::memcpy(pGazeImage, ptr, 0x200100);
    }
  }

  void CAPI_WritePcm(VRControllerType controllerType, const unsigned char* pcm) {
    if (g_slot >= 0) CustomShareManager::getSingleton()->writePcm(g_slot, controllerType, pcm);
  }

  void CAPI_WaitForPcmUpdate() {
    if (g_slot >= 0) CustomShareManager::getSingleton()->waitForPcmUpdate(g_slot);
  }

  void CAPI_SendTriggerEffect(const TriggerEffectCommandPayload* payload) {
    if (g_slot >= 0) CustomShareManager::getSingleton()->pushTriggerEffect(g_slot, *payload);
  }

}