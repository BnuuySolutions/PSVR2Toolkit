add_library(psvr2_toolkit_capi_loader SHARED
  psvr2tk_capi_loader.cpp
  psvr2tk_capi_loader.h
  psvr2tk_capi_loader_private.h
)

target_include_directories(psvr2_toolkit_capi_loader PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(psvr2_toolkit_capi_loader PUBLIC psvr2_toolkit_capi_headers)
