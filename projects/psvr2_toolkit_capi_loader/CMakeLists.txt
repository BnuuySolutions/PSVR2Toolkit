add_library(psvr2_toolkit_capi_loader SHARED
  psvr2tk_capi_loader.cpp
)
target_include_directories(psvr2_toolkit_capi_loader PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(psvr2_toolkit_capi_loader PRIVATE libcustomshare)

target_include_directories(psvr2_toolkit_capi PUBLIC ${CMAKE_SOURCE_DIR}/include)
