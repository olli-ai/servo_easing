add_executable(servo_easing_test ${CMAKE_CURRENT_SOURCE_DIR}/test_servo_easing.c)

target_include_directories(servo_easing_test PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_include_directories(servo_easing_test PRIVATE ${PROJECT_SOURCE_DIR}/3rd_party/logging)
target_include_directories(servo_easing_test PRIVATE ${PROJECT_SOURCE_DIR}/internal)
target_link_libraries(servo_easing_test ${PROJECT_NAME})
if(USE_FLOAT)
target_link_libraries(servo_easing_test m)
endif(USE_FLOAT)