add_library(customshare SHARED
  customshare.cpp
)
target_include_directories(customshare PRIVATE .)

target_link_libraries(customshare PRIVATE libcustomshare)
