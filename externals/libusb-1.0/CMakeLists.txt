add_library(libusb INTERFACE)
target_include_directories(libusb INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")

# Add libusb-1.0.lib
target_link_libraries(libusb INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/libusb-1.0.lib")

# Copy dll to bin output
if (LINUX)
  set(LIBUSB_DLL_SRC "${CMAKE_CURRENT_SOURCE_DIR}/libusb-1.0-winedll.dll")
else()
  set(LIBUSB_DLL_SRC "${CMAKE_CURRENT_SOURCE_DIR}/libusb-1.0.dll")
endif()

if (MSVC)
  set(LIBUSB_DLL_DEST "${CMAKE_BINARY_DIR}/bin/$<CONFIG>/libusb-1.0.dll")
else()
  set(LIBUSB_DLL_DEST "${CMAKE_BINARY_DIR}/bin/libusb-1.0.dll")
endif()

add_custom_target(libusb_copy_dll ALL
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${LIBUSB_DLL_SRC}"
    "${LIBUSB_DLL_DEST}"
  COMMENT "Copying libusb-1.0.dll to output directory"
)