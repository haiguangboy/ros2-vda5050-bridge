cmake_minimum_required(VERSION 3.16)

# Enable testing
enable_testing()

# Simplified tests without gtest dependency for now

# Test for Real /plan Topic Subscription
add_executable(test_real_plan_subscription
    test_real_plan_subscription.cpp
)

target_link_libraries(test_real_plan_subscription
    zhongli_protocol_types
    zhongli_mqtt_client
    ros2_zhongli_bridge
    nlohmann_json::nlohmann_json
)

# Add MQTT libraries if available
if(MQTT_ENABLED)
    target_link_libraries(test_real_plan_subscription ${MQTT_LIBRARIES})
endif()

# ROS2 dependencies for real subscription test
ament_target_dependencies(test_real_plan_subscription
    rclcpp
    nav_msgs
    geometry_msgs
    tf2_ros
    tf2_geometry_msgs
)

# Set the RPATH for test executables
set_target_properties(test_real_plan_subscription PROPERTIES
    INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
    INSTALL_RPATH_USE_LINK_PATH TRUE
)

# Install test executables to cpp_version/install
install(TARGETS
    test_real_plan_subscription
    DESTINATION bin
)