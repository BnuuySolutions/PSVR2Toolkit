add_executable(psvr2_gaze_filter_test main.cpp)

target_link_libraries(psvr2_gaze_filter_test PRIVATE common)

add_test(NAME gaze_filter COMMAND psvr2_gaze_filter_test)
