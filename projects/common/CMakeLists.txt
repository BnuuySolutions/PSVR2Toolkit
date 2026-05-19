add_library(common INTERFACE
  common.h)
target_include_directories(common PUBLIC INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})