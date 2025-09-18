#include <rclcpp/rclcpp.hpp>
#include "ros2_zhongli_bridge.hpp"
#include <signal.h>
#include <iostream>

// 全局桥接器指针，用于信号处理
std::shared_ptr<zhongli_bridge::ROS2ZhongliBridge> g_bridge = nullptr;

void signal_handler(int signum) {
    std::cout << std::endl << "🛑 收到终止信号 (" << signum << ")，正在关闭桥接器..." << std::endl;

    if (g_bridge) {
        g_bridge->stop();
    }

    rclcpp::shutdown();
    exit(signum);
}

int main(int argc, char** argv) {
    // 初始化ROS2
    rclcpp::init(argc, argv);

    // 设置信号处理
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    std::cout << "🚀 中力具身机器人ROS2桥接器 - C++版本" << std::endl;
    std::cout << "===============================================" << std::endl;

    try {
        // 配置节点选项以加载参数文件
        rclcpp::NodeOptions node_options;

        // 检查是否通过命令行参数指定了配置文件
        std::string config_file_path = "../config/bridge_config.yaml";
        for (int i = 1; i < argc; i++) {
            std::string arg(argv[i]);
            if (arg.find("--ros-args") != std::string::npos) {
                // 如果有 --ros-args，让ROS2自己处理参数
                break;
            } else if (arg.find("--config") != std::string::npos && i + 1 < argc) {
                config_file_path = argv[i + 1];
                i++; // 跳过下一个参数
            }
        }

        // 尝试设置参数文件
        std::cout << "📋 尝试加载配置文件: " << config_file_path << std::endl;

        // 创建桥接器节点
        g_bridge = std::make_shared<zhongli_bridge::ROS2ZhongliBridge>(node_options);

        // 初始化桥接器
        if (!g_bridge->initialize()) {
            std::cerr << "❌ 桥接器初始化失败" << std::endl;
            return -1;
        }

        // 启动桥接器
        g_bridge->start();

        std::cout << "🎯 桥接器启动成功，正在运行..." << std::endl;
        std::cout << "按 Ctrl+C 停止桥接器" << std::endl;

        // 运行ROS2事件循环
        rclcpp::spin(g_bridge);

    } catch (const std::exception& e) {
        std::cerr << "❌ 运行错误: " << e.what() << std::endl;
        return -1;
    }

    // 清理
    if (g_bridge) {
        g_bridge->stop();
        g_bridge.reset();
    }

    rclcpp::shutdown();

    std::cout << "✅ 桥接器已正常退出" << std::endl;
    return 0;
}