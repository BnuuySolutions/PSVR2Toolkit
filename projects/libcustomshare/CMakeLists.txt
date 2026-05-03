add_library(libcustomshare STATIC
  custom_share_manager.cpp
)
target_include_directories(libcustomshare PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Require libcrossipc
target_link_libraries(libcustomshare PUBLIC libcrossipc)
# And its headers
target_include_directories(libcustomshare PUBLIC ${CMAKE_SOURCE_DIR}/projects/libcrossipc)

target_include_directories(libcustomshare PUBLIC ${CMAKE_SOURCE_DIR}/include)
