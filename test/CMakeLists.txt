find_package(GTest REQUIRED)

add_executable(${PROJECT_NAME}_test_checkup_attitude test_checkup_attitude.cpp )
target_link_libraries(${PROJECT_NAME}_test_checkup_attitude ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_checkup_attitude PRIVATE -std=c++17)
add_test(test_checkup_attitude ${PROJECT_NAME}_test_checkup_attitude)


add_executable(${PROJECT_NAME}_test_angular_speed_bias test_angular_speed_bias.cpp )
target_link_libraries(${PROJECT_NAME}_test_angular_speed_bias ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_angular_speed_bias PRIVATE -std=c++17)
add_test(test_angular_speed_bias ${PROJECT_NAME}_test_angular_speed_bias)

#catkin_add_gtest(${PROJECT_NAME}_test_gps_plugin test_gps_plugin.cpp )
#target_link_libraries(${PROJECT_NAME}_test_gps_plugin ${PROJECT_NAME})
