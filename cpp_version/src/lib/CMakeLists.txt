cmake_minimum_required(VERSION 3.16)

# Zhongli Protocol Types Library
add_library(zhongli_protocol_types SHARED
    zhongli_protocol_types.cpp
)

target_include_directories(zhongli_protocol_types PUBLIC
    ../include
)

target_link_libraries(zhongli_protocol_types
    nlohmann_json::nlohmann_json
)

# MQTT Client Library (conditional on MQTT availability)
if(MQTT_ENABLED)
    add_library(zhongli_mqtt_client SHARED
        zhongli_mqtt_client.cpp
    )

    target_include_directories(zhongli_mqtt_client PUBLIC
        ../include
    )

    target_link_libraries(zhongli_mqtt_client
        zhongli_protocol_types
        ${MQTT_LIBRARIES}
        nlohmann_json::nlohmann_json
    )
else()
    # Create a dummy MQTT client library for compilation
    add_library(zhongli_mqtt_client SHARED
        zhongli_mqtt_client_dummy.cpp
    )

    target_include_directories(zhongli_mqtt_client PUBLIC
        ../include
    )

    target_link_libraries(zhongli_mqtt_client
        zhongli_protocol_types
        nlohmann_json::nlohmann_json
    )
endif()

# ROS2 Bridge Library
add_library(ros2_zhongli_bridge SHARED
    ros2_zhongli_bridge.cpp
    path_converter.cpp
)

# Set RPATH for shared libraries
set_target_properties(zhongli_protocol_types PROPERTIES
    INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
    INSTALL_RPATH_USE_LINK_PATH TRUE
)

set_target_properties(zhongli_mqtt_client PROPERTIES
    INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
    INSTALL_RPATH_USE_LINK_PATH TRUE
)

set_target_properties(ros2_zhongli_bridge PROPERTIES
    INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
    INSTALL_RPATH_USE_LINK_PATH TRUE
)

target_include_directories(ros2_zhongli_bridge PUBLIC
    ../include
)

target_link_libraries(ros2_zhongli_bridge
    zhongli_protocol_types
    zhongli_mqtt_client
)

ament_target_dependencies(ros2_zhongli_bridge
    rclcpp
    std_msgs
    geometry_msgs
    nav_msgs
    nav2_msgs
    tf2_ros
    tf2_geometry_msgs
)