add_definitions(-DNOMINMAX)

add_library(psvr2_openvr_driver_ex SHARED
  caesar_manager_hooks.cpp
  device_provider_proxy.cpp
  driver_context_proxy.cpp
  driver_host_proxy.cpp
  hmd_device_hooks.cpp
  hmd_driver_factory.cpp
  hmd_driver_loader.cpp
  libpad_hooks.cpp
  ipc_server.cpp
  sense_controller.cpp
  trigger_effect_manager.cpp
  usb_thread_gaze.cpp
  usb_thread_hooks.cpp
)
set_target_properties(psvr2_openvr_driver_ex PROPERTIES
  OUTPUT_NAME "driver_playstation_vr2"
)
target_include_directories(psvr2_openvr_driver_ex PRIVATE .)
target_include_directories(psvr2_openvr_driver_ex PRIVATE ./psvr2_openvr_driver/openvr/headers)
target_link_libraries(psvr2_openvr_driver_ex PRIVATE shlwapi ws2_32 winusb)
target_link_libraries(psvr2_openvr_driver_ex PRIVATE PolyHook_2)
