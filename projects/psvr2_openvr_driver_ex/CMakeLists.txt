add_definitions(-DNOMINMAX)

add_library(psvr2_openvr_driver_ex SHARED
  driver_hooks/aston_manager_hooks.cpp
  driver_hooks/caesar_manager_hooks.h
  driver_hooks/caesar_manager_hooks.cpp
  driver_hooks/hmd_device_hooks.h
  driver_hooks/hmd_device_hooks.cpp
  driver_hooks/libpad_hooks.h
  driver_hooks/libpad_hooks.cpp
  driver_hooks/sense_device_hooks.cpp
  driver_hooks/usb_thread_hooks.h
  driver_hooks/usb_thread_hooks.cpp

  driver_interface/aston_context.h
  driver_interface/aston_manager.h
  driver_interface/caesar_manager.h
  driver_interface/share_manager.h
  driver_interface/share_manager.cpp

  caesar_usb_thread.h
  caesar_usb_thread.cpp
  command_thread.h
  command_thread.cpp
  device_provider_proxy.h
  device_provider_proxy.cpp
  driver_context_proxy.h
  driver_context_proxy.cpp
  driver_host_proxy.h
  driver_host_proxy.cpp
  hmd_device_camera.h
  hmd_device_camera.cpp
  hmd_driver_factory.cpp
  hmd_driver_loader.h
  hmd_driver_loader.cpp
  hmd_math.h
  hmd_types.h
  hook_lib.h
  img_utils.h
  math_helpers.h
  openvr_driver_internal.h
  resources_patch.h
  resources_patch.cpp
  sense_controller.h
  sense_controller.cpp
  sense_crc.h
  trigger_effect_manager.h
  trigger_effect_manager.cpp
  usb_thread_gaze.h
  usb_thread_gaze.cpp
  util.h
  vr_settings.h
  write_file_async.h
)
target_include_directories(psvr2_openvr_driver_ex PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(psvr2_openvr_driver_ex PRIVATE shlwapi hid)
target_link_libraries(psvr2_openvr_driver_ex PRIVATE common libcustomshare)

# Link our external libraries
target_link_libraries(psvr2_openvr_driver_ex PRIVATE PolyHook_2)
target_link_libraries(psvr2_openvr_driver_ex PRIVATE openvr-headers)
target_link_libraries(psvr2_openvr_driver_ex PRIVATE libusb)

# DirectX 11 for GPU compute based BC4 to NV12 conversion
target_link_libraries(psvr2_openvr_driver_ex PRIVATE d3d11)
target_link_libraries(psvr2_openvr_driver_ex PRIVATE d3dcompiler)

set_target_properties(psvr2_openvr_driver_ex PROPERTIES
  OUTPUT_NAME driver_playstation_vr2
)
