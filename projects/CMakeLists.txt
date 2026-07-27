add_subdirectory(common)
add_subdirectory(libcrossipc)
add_subdirectory(libcustomshare)
if (WIN32)
  add_subdirectory(psvr2_openvr_driver_ex)
endif()
add_subdirectory(psvr2_toolkit_capi)
add_subdirectory(psvr2_toolkit_capi_loader)
add_subdirectory(psvr2_toolkit_capi_test)
add_subdirectory(psvr2_toolkit_app)
add_subdirectory(psvr2_toolkit_legacy_ipc_server)
if (WIN32)
  add_subdirectory(psvr2_toolkit_redir)
endif()

set(CAPI_DIST_DIR "${CMAKE_BINARY_DIR}/capi_dist")

add_custom_target(psvr2_toolkit_capi_dist ALL
  COMMAND ${CMAKE_COMMAND} -E make_directory "${CAPI_DIST_DIR}/include"
  COMMAND ${CMAKE_COMMAND} -E make_directory "${CAPI_DIST_DIR}/lib"
  COMMAND ${CMAKE_COMMAND} -E make_directory "${CAPI_DIST_DIR}/bin"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different 
    "${CMAKE_CURRENT_SOURCE_DIR}/psvr2_toolkit_capi_loader/psvr2tk_capi_loader.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/psvr2_toolkit_capi/psvr2tk_capi.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/common.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/hmd2_gaze.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/pad_trigger_effect.h"
    "${CAPI_DIST_DIR}/include/"
  COMMENT "Copying files to CAPI dist folder"
)

if(WIN32)
  add_custom_command(TARGET psvr2_toolkit_capi_dist POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "$<TARGET_FILE:psvr2_toolkit_capi_loader>" "${CAPI_DIST_DIR}/bin/"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "$<TARGET_LINKER_FILE:psvr2_toolkit_capi_loader>" "${CAPI_DIST_DIR}/lib/"
  )
else()
  add_custom_command(TARGET psvr2_toolkit_capi_dist POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "$<TARGET_FILE:psvr2_toolkit_capi_loader>" "${CAPI_DIST_DIR}/lib/"
  )
endif()


if(NOT CMAKE_HOST_WIN32)
    # Also build libcrossipc as separate project to make the winedll
    include(ExternalProject)
    externalproject_add(
        libcrossipc_wine
        SOURCE_DIR      "${CMAKE_CURRENT_SOURCE_DIR}/libcrossipc"
        BINARY_DIR      "${CMAKE_CURRENT_BINARY_DIR}/libcrossipc_build"
        INSTALL_DIR     "${CMAKE_BINARY_DIR}/winedll"
        CMAKE_ARGS      -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
                        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
                        -DCMAKE_TOOLCHAIN_FILE=${CMAKE_CURRENT_SOURCE_DIR}/../wine.cmake
        BUILD_ALWAYS    1
        INSTALL_COMMAND ${CMAKE_COMMAND} -E copy <BINARY_DIR>/libcrossipc.so <INSTALL_DIR>/libcrossipc.dll
    )
endif()
