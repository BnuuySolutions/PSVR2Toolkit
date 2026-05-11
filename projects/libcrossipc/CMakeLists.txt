cmake_minimum_required(VERSION 4.3)

if(WIN32)
  set(PLATFORM_IPC_SOURCES windows_ipc.cpp)
else()
  set(PLATFORM_IPC_SOURCES linux_ipc.cpp)
endif()

add_library(libcrossipc SHARED
  cross_ipc.cpp
  ${PLATFORM_IPC_SOURCES}
)

target_include_directories(libcrossipc PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

if(UNIX)
  target_link_libraries(libcrossipc PRIVATE rt pthread)
endif()

if (USING_WINE)
  target_link_libraries(libcrossipc PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/cross_ipc.spec)
endif()
