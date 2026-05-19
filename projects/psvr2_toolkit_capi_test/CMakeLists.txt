cmake_minimum_required(VERSION 3.10)
project(psvr2_toolkit_capi_test)

set(CMAKE_CXX_STANDARD 17)

add_executable(psvr2_toolkit_capi_test main.cpp)

target_link_libraries(psvr2_toolkit_capi_test PRIVATE SDL3::SDL3 imgui)

target_link_libraries(psvr2_toolkit_capi_test PRIVATE psvr2_toolkit_capi_loader)