add_subdirectory(PolyHook_2_0)

add_library(openvr-headers INTERFACE)
target_include_directories(openvr-headers INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/openvr/headers")
