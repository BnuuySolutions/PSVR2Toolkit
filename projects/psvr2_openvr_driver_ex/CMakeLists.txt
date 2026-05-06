add_definitions(-DNOMINMAX)

add_library(psvr2_openvr_driver_ex SHARED
  common/hmd2_gaze.h

  driver_hooks/caesar_manager_hooks.h
  driver_hooks/caesar_manager_hooks.cpp
  driver_hooks/hmd_device_hooks.h
  driver_hooks/hmd_device_hooks.cpp
  driver_hooks/libpad_hooks.h
  driver_hooks/libpad_hooks.cpp
  driver_hooks/usb_thread_hooks.h
  driver_hooks/usb_thread_hooks.cpp

  driver_interface/aston_context.h
  driver_interface/aston_manager.h

  caesar_usb_thread.cpp
  command_thread.cpp
  device_provider_proxy.cpp
  driver_context_proxy.cpp
  driver_host_proxy.cpp
  hmd_driver_factory.cpp
  hmd_driver_loader.cpp
  sense_controller.cpp
  trigger_effect_manager.cpp
  usb_thread_gaze.cpp
)
target_include_directories(psvr2_openvr_driver_ex PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(psvr2_openvr_driver_ex PUBLIC ${CMAKE_SOURCE_DIR}/include)

target_link_libraries(psvr2_openvr_driver_ex PRIVATE shlwapi winusb)
target_link_libraries(psvr2_openvr_driver_ex PRIVATE libcustomshare)

# Link our external libraries
target_link_libraries(psvr2_openvr_driver_ex PRIVATE PolyHook_2)
target_link_libraries(psvr2_openvr_driver_ex PRIVATE openvr-headers)
target_link_libraries(psvr2_openvr_driver_ex PRIVATE libusb)

set_target_properties(psvr2_openvr_driver_ex PROPERTIES
  OUTPUT_NAME driver_playstation_vr2
)
