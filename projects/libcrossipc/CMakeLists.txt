add_library(libcrossipc SHARED
  cross_ipc.cpp
  windows_ipc.cpp
)

target_include_directories(libcrossipc PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
