set(EXECUTABLE_OPTIONS)

if(WIN32)
  set(EXECUTABLE_OPTIONS WIN32)
endif()

add_executable(psvr2_toolkit_app ${EXECUTABLE_OPTIONS}
  main.cpp
)
target_include_directories(psvr2_toolkit_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# TODO(bnuuy): We should consider switching to SDL3 shared library in the future.
target_link_libraries(psvr2_toolkit_app PRIVATE SDL3::SDL3-static imgui)
