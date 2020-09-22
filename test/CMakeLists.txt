include_directories(${CMAKE_CURRENT_BINARY_DIR})

catkin_add_gtest(${PROJECT_NAME}_test_checkup_attitude test_checkup_attitude.cpp )
target_link_libraries(${PROJECT_NAME}_test_checkup_attitude ${PROJECT_NAME})

catkin_add_gtest(${PROJECT_NAME}_test_angular_speed_bias test_angular_speed_bias.cpp )
target_link_libraries(${PROJECT_NAME}_test_angular_speed_bias ${PROJECT_NAME})

#catkin_add_gtest(${PROJECT_NAME}_test_gps_plugin test_gps_plugin.cpp )
#target_link_libraries(${PROJECT_NAME}_test_gps_plugin ${PROJECT_NAME})
