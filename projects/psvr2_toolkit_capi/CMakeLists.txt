add_library(psvr2_toolkit_capi SHARED
  capi.cpp
)
target_include_directories(psvr2_toolkit_capi PRIVATE .)

target_link_libraries(psvr2_toolkit_capi PRIVATE libcustomshare)

# TODO: Make this output name Windows-only.
set_target_properties(psvr2_toolkit_capi PROPERTIES
  OUTPUT_NAME PSVR2Toolkit.CAPI
)
