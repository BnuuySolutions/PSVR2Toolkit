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
