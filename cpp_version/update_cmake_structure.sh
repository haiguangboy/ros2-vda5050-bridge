#!/bin/bash
# update_cmake_structure.sh
# 更新CMakeLists.txt以适应新的目录结构

echo "🔧 更新CMakeLists.txt结构"
echo "========================"

# 备份原始CMakeLists.txt
if [ -f "CMakeLists.txt" ]; then
    cp CMakeLists.txt CMakeLists.txt.backup
    echo "✅ 已备份原始CMakeLists.txt"
fi

# 更新测试部分的路径
if [ -f "CMakeLists.txt" ]; then
    # 更新测试目录路径
    sed -i 's|add_subdirectory(src/tests)|# Tests moved to tests/ directory\n# add_subdirectory(tests)|g' CMakeLists.txt
    echo "✅ 已更新CMakeLists.txt中的测试路径"
fi

# 创建tests目录的CMakeLists.txt
cat > tests/CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.16)

# 单元测试
if(BUILD_TESTING)
    # 路径转换测试
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/unit/test_real_path_conversion.cpp)
        add_executable(test_path_conversion
            unit/test_real_path_conversion.cpp
        )

        target_link_libraries(test_path_conversion
            zhongli_protocol_types
            zhongli_mqtt_client
            ros2_zhongli_bridge
            nlohmann_json::nlohmann_json
        )

        ament_target_dependencies(test_path_conversion
            rclcpp
            nav_msgs
            geometry_msgs
            tf2_ros
            tf2_geometry_msgs
        )

        set_target_properties(test_path_conversion PROPERTIES
            INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
            INSTALL_RPATH_USE_LINK_PATH TRUE
        )

        install(TARGETS test_path_conversion
            DESTINATION bin
        )
    endif()

    # 简单路径测试
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/unit/test_path_simple.cpp)
        add_executable(test_path_simple
            unit/test_path_simple.cpp
        )

        target_link_libraries(test_path_simple
            zhongli_protocol_types
            nlohmann_json::nlohmann_json
        )

        set_target_properties(test_path_simple PROPERTIES
            INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
            INSTALL_RPATH_USE_LINK_PATH TRUE
        )

        install(TARGETS test_path_simple
            DESTINATION bin
        )
    endif()
endif()

# 安装测试脚本
install(DIRECTORY scripts/
    DESTINATION share/${PROJECT_NAME}/tests/scripts/
    USE_SOURCE_PERMISSIONS
    FILES_MATCHING PATTERN "*.sh"
)

install(DIRECTORY integration/
    DESTINATION share/${PROJECT_NAME}/tests/integration/
    USE_SOURCE_PERMISSIONS
    FILES_MATCHING PATTERN "*.sh"
)

install(FILES run_all_tests.sh
    DESTINATION share/${PROJECT_NAME}/tests/
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)
EOF

echo "✅ 已创建tests/CMakeLists.txt"

# 更新主CMakeLists.txt以包含新的测试结构
if [ -f "CMakeLists.txt" ]; then
    # 在文件末尾添加新的测试目录
    cat >> CMakeLists.txt << 'EOF'

# 新的测试结构
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
EOF
    echo "✅ 已更新主CMakeLists.txt"
fi

echo ""
echo "🎯 CMakeLists.txt结构更新完成"
echo "  ✅ 备份了原始文件"
echo "  ✅ 更新了测试目录路径"
echo "  ✅ 创建了新的tests/CMakeLists.txt"
echo "  ✅ 配置了测试脚本安装"