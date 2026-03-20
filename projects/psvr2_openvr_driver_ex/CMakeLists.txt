add_library(psvr2_openvr_driver_ex SHARED
  caesar_manager_hooks.cpp
  device_provider_proxy.cpp
  driver_context_proxy.cpp
  driver_host_proxy.cpp
  eyelid_estimator.cpp
  hmd_device_hooks.cpp
  hmd_driver_factory.cpp
  hmd_driver_loader.cpp
  ipc_server.cpp
  trigger_effect_manager.cpp
  usb_thread_gaze.cpp
  usb_thread_hooks.cpp
)
set_target_properties(psvr2_openvr_driver_ex PROPERTIES
  OUTPUT_NAME "psvr2_openvr_driver"
)
target_include_directories(psvr2_openvr_driver_ex PRIVATE .)
target_include_directories(psvr2_openvr_driver_ex PRIVATE ./minhook/include)
target_include_directories(psvr2_openvr_driver_ex PRIVATE ./psvr2_openvr_driver/openvr/headers)
target_link_libraries(psvr2_openvr_driver_ex PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/minhook/lib/Release/libMinHook.x64.lib")
target_link_libraries(psvr2_openvr_driver_ex PRIVATE shlwapi ws2_32 winusb)
