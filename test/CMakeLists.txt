find_package(GTest REQUIRED)
include(romea_add_gtest)

romea_add_gtest(test_checkup_attitude test_checkup_attitude.cpp LIBRARIES ${PROJECT_NAME})

romea_add_gtest(test_checkup_inertial_measurements test_checkup_inertial_measurements.cpp LIBRARIES ${PROJECT_NAME})

romea_add_gtest(test_angular_speed_bias test_angular_speed_bias.cpp LIBRARIES ${PROJECT_NAME})

romea_add_gtest(test_imu_plugin test_imu_plugin.cpp LIBRARIES ${PROJECT_NAME})
