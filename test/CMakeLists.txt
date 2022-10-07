find_package(GTest REQUIRED)

add_executable(${PROJECT_NAME}_test_checkup_attitude test_checkup_attitude.cpp )
target_link_libraries(${PROJECT_NAME}_test_checkup_attitude ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_checkup_attitude PRIVATE -std=c++17)
add_test(test_checkup_attitude ${PROJECT_NAME}_test_checkup_attitude)

add_executable(${PROJECT_NAME}_test_checkup_inertial_measurements test_checkup_inertial_measurements.cpp )
target_link_libraries(${PROJECT_NAME}_test_checkup_inertial_measurements ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_checkup_inertial_measurements PRIVATE -std=c++17)
add_test(test_checkup_attitude ${PROJECT_NAME}_test_checkup_inertial_measurements)

add_executable(${PROJECT_NAME}_test_angular_speed_bias test_angular_speed_bias.cpp )
target_link_libraries(${PROJECT_NAME}_test_angular_speed_bias ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_angular_speed_bias PRIVATE -std=c++17)
add_test(test_angular_speed_bias ${PROJECT_NAME}_test_angular_speed_bias)

add_executable(${PROJECT_NAME}_test_imu_plugin test_imu_plugin.cpp )
target_link_libraries(${PROJECT_NAME}_test_imu_plugin  ${PROJECT_NAME} GTest::GTest GTest::Main)
target_compile_options(${PROJECT_NAME}_test_imu_plugin  PRIVATE -std=c++17)
add_test(test_imu_plugin  ${PROJECT_NAME}_test_imu_plugin )

