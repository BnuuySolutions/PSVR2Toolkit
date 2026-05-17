#include "sense_device_hooks.h"

#include "driver_host_proxy.h"
#include "hmd_driver_loader.h"
#include "hmd_types.h"
#include "hook_lib.h"

namespace psvr2_toolkit {

  vr::EVRInitError(*sie__psvr2__SenseDevice__Activate)(void *, uint32_t) = nullptr;
  vr::EVRInitError sie__psvr2__SenseDevice__ActivateHook(void *thisptr, uint32_t unObjectId) {
    vr::EVRInitError result = sie__psvr2__SenseDevice__Activate(thisptr, unObjectId);
    vr::PropertyContainerHandle_t ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(unObjectId);

    if (*(int *)((uint64_t)thisptr + 0xc) == 0) {
      DriverHostProxy::Instance()->SetDevice(DeviceType::SenseControllerLeft, ulPropertyContainer, unObjectId);
    }
    else {
      DriverHostProxy::Instance()->SetDevice(DeviceType::SenseControllerRight, ulPropertyContainer, unObjectId);
    }

    return result;
  }

  void (*sie__psvr2__SenseDevice__Deactivate)(void *) = nullptr;
  void sie__psvr2__SenseDevice__DeactivateHook(void *thisptr) {
    sie__psvr2__SenseDevice__Deactivate(thisptr);

    if (*(int *)((uint64_t)thisptr + 0xc) == 0) {
      DriverHostProxy::Instance()->SetDevice(DeviceType::SenseControllerLeft, vr::k_ulInvalidPropertyContainer, vr::k_unTrackedDeviceIndexInvalid);
    }
    else {
      DriverHostProxy::Instance()->SetDevice(DeviceType::SenseControllerRight, vr::k_ulInvalidPropertyContainer, vr::k_unTrackedDeviceIndexInvalid);
    }
  }

  void SenseDeviceHooks::InstallHooks() {
    static HmdDriverLoader *pHmdDriverLoader = HmdDriverLoader::Instance();

    // sie::psvr2::SenseDevice::Activate
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x1A5760),
                         reinterpret_cast<void *>(sie__psvr2__SenseDevice__ActivateHook),
                         reinterpret_cast<void **>(&sie__psvr2__SenseDevice__Activate));

    // sie::psvr2::SenseDevice::Deactivate
    HookLib::InstallHook(reinterpret_cast<void *>(pHmdDriverLoader->GetBaseAddress() + 0x1A7a50),
                         reinterpret_cast<void *>(sie__psvr2__SenseDevice__DeactivateHook),
                         reinterpret_cast<void **>(&sie__psvr2__SenseDevice__Deactivate));
  }

} // psvr2_toolkit
