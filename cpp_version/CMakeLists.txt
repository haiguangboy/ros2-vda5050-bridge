cmake_minimum_required(VERSION 3.16)
project(ros2_zhongli_bridge_cpp)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set install prefix to cpp_version/install
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/install" CACHE PATH "Install prefix" FORCE)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

# Find nlohmann/json
find_package(nlohmann_json REQUIRED)

# Check for mosquitto MQTT client library and headers (for connecting to EMQX broker)
# 支持多架构：自动检测系统架构
execute_process(COMMAND dpkg --print-architecture
                OUTPUT_VARIABLE SYSTEM_ARCH
                OUTPUT_STRIP_TRAILING_WHITESPACE)

set(MOSQUITTO_LIB_PATH "/usr/lib/${SYSTEM_ARCH}-linux-gnu/libmosquitto.so.1")
find_path(MOSQUITTO_INCLUDE_DIR mosquitto.h PATHS /usr/include /usr/local/include)

# 尝试查找mosquitto库文件
if(EXISTS ${MOSQUITTO_LIB_PATH} AND MOSQUITTO_INCLUDE_DIR)
    message(STATUS "Found mosquitto MQTT client library: ${MOSQUITTO_LIB_PATH}")
    message(STATUS "Found mosquitto headers: ${MOSQUITTO_INCLUDE_DIR}")
    set(MQTT_ENABLED TRUE)
    set(MQTT_LIBRARIES ${MOSQUITTO_LIB_PATH})
    include_directories(${MOSQUITTO_INCLUDE_DIR})
    add_definitions(-DMQTT_ENABLED)
else()
    # 如果找不到，尝试使用pkg-config查找
    find_package(PkgConfig QUIET)
    if(PKG_CONFIG_FOUND)
        pkg_check_modules(MOSQUITTO QUIET libmosquitto)
        if(MOSQUITTO_FOUND)
            message(STATUS "Found mosquitto via pkg-config: ${MOSQUITTO_LIBRARIES}")
            message(STATUS "Found mosquitto headers: ${MOSQUITTO_INCLUDE_DIRS}")
            set(MQTT_ENABLED TRUE)
            set(MQTT_LIBRARIES ${MOSQUITTO_LIBRARIES})
            include_directories(${MOSQUITTO_INCLUDE_DIRS})
            add_definitions(-DMQTT_ENABLED)
        else()
            message(WARNING "mosquitto library not found via pkg-config either")
            set(MQTT_ENABLED FALSE)
        endif()
    else()
        message(WARNING "mosquitto library or headers not found, using dummy MQTT client")
        message(STATUS "Searched path: ${MOSQUITTO_LIB_PATH}")
        message(STATUS "Headers found: ${MOSQUITTO_INCLUDE_DIR}")
        set(MQTT_ENABLED FALSE)
    endif()
endif()

# Include directories
include_directories(src/include)

# Add subdirectories
add_subdirectory(src/lib)

# Create main executable
add_executable(zhongli_bridge_node
    src/main.cpp
)

# Link libraries
target_link_libraries(zhongli_bridge_node
    zhongli_protocol_types
    zhongli_mqtt_client
    ros2_zhongli_bridge
    nlohmann_json::nlohmann_json
)

# Add MQTT libraries if enabled
if(MQTT_ENABLED)
    target_link_libraries(zhongli_bridge_node ${MQTT_LIBRARIES})
endif()

# ROS2 dependencies
ament_target_dependencies(zhongli_bridge_node
    rclcpp
    std_msgs
    geometry_msgs
    nav_msgs
    nav2_msgs
    tf2_ros
    tf2_geometry_msgs
)

# Set the RPATH for the executable
set_target_properties(zhongli_bridge_node PROPERTIES
    INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
    INSTALL_RPATH_USE_LINK_PATH TRUE
)

# Install main executable and libraries
install(TARGETS zhongli_bridge_node
    DESTINATION bin
)

# Install shared libraries
install(TARGETS zhongli_protocol_types zhongli_mqtt_client ros2_zhongli_bridge
    DESTINATION lib
)

# Install config files
install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config/
    OPTIONAL
)

# Testing
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)

    # Add test subdirectory
    add_subdirectory(src/tests)
endif()

ament_package()