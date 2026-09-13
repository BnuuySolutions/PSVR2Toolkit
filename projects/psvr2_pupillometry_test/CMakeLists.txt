add_executable(psvr2_pupillometry_test main.cpp)

target_link_libraries(psvr2_pupillometry_test PRIVATE common)

add_test(NAME pupillometry COMMAND psvr2_pupillometry_test)
