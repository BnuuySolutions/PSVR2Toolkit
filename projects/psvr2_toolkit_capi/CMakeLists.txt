add_library(psvr2_toolkit_capi SHARED
  capi.cpp
)
target_include_directories(psvr2_toolkit_capi PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(psvr2_toolkit_capi PRIVATE libcustomshare)
