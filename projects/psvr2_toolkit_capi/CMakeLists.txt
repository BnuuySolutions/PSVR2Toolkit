add_library(psvr2_toolkit_capi SHARED
  psvr2tk_capi.cpp
  psvr2tk_capi.h
)
target_include_directories(psvr2_toolkit_capi PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(psvr2_toolkit_capi PRIVATE libcustomshare)

target_include_directories(psvr2_toolkit_capi PUBLIC ${CMAKE_SOURCE_DIR}/include)
