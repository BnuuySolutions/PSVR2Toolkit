set(EXECUTABLE_OPTIONS)

if(WIN32)
  set(EXECUTABLE_OPTIONS WIN32)
endif()

add_executable(psvr2_toolkit_app ${EXECUTABLE_OPTIONS}
  main.cpp
)
target_include_directories(psvr2_toolkit_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(psvr2_toolkit_app PRIVATE SDL3::SDL3 imgui)

# Make sure we copy the SDL3 shared library to the output directory.
add_custom_command(TARGET psvr2_toolkit_app POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    $<TARGET_FILE:SDL3::SDL3>
    $<TARGET_FILE_DIR:psvr2_toolkit_app>
)
