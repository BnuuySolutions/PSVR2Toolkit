add_library(libcustomshare STATIC
  custom_share_manager.cpp
)
target_include_directories(libcustomshare PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
