add_executable(psvr2_toolkit_legacy_ipc_server
  main.cpp
)
target_include_directories(psvr2_toolkit_legacy_ipc_server PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(psvr2_toolkit_legacy_ipc_server PRIVATE psvr2_toolkit_capi_loader)
