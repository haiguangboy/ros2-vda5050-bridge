#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "path_converter.hpp"
#include "zhongli_protocol_types.hpp"
#include <iostream>
#include <iomanip>

using namespace zhongli_bridge;

/**
 * @brief 创建与真实Nav2 /plan 话题相同格式的测试路径
 */
nav_msgs::msg::Path create_real_nav2_path() {
    nav_msgs::msg::Path path;

    // 设置header
    path.header.stamp.sec = 1757908301;
    path.header.stamp.nanosec = 171455323;
    path.header.frame_id = "map";

    // 创建真实的路径点数据（基于您提供的/plan话题数据）
    std::vector<std::tuple<double, double, double, double, double, double, double>> waypoints = {
        // x, y, z, qx, qy, qz, qw
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0},           // 起点，朝东(0度)
        {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0},           // 东移1米
        {2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0},           // 继续东移
        {3.0, 1.0, 0.0, 0.0, 0.0, 0.3826834, 0.9238795}, // 转向东北
        {3.0, 2.0, 0.0, 0.0, 0.0, 0.7071068, 0.7071068}  // 转向北(90度)
    };

    for (const auto& wp : waypoints) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;

        pose.pose.position.x = std::get<0>(wp);
        pose.pose.position.y = std::get<1>(wp);
        pose.pose.position.z = std::get<2>(wp);

        pose.pose.orientation.x = std::get<3>(wp);
        pose.pose.orientation.y = std::get<4>(wp);
        pose.pose.orientation.z = std::get<5>(wp);
        pose.pose.orientation.w = std::get<6>(wp);

        path.poses.push_back(pose);
    }

    return path;
}

/**
 * @brief 验证四元数到角度转换的精确性
 */
void test_quaternion_conversion() {
    std::cout << "🧮 测试四元数到角度转换精确性" << std::endl;
    std::cout << "=================================" << std::endl;

    // 测试用例：已知的四元数和对应角度
    std::vector<std::tuple<double, double, double, double, double>> test_cases = {
        // qx, qy, qz, qw, expected_yaw_degrees
        {0.0, 0.0, 0.0, 1.0, 0.0},                    // 0度
        {0.0, 0.0, 0.3826834, 0.9238795, 45.0},       // 45度
        {0.0, 0.0, 0.7071068, 0.7071068, 90.0},       // 90度
        {0.0, 0.0, 0.9238795, 0.3826834, 135.0},      // 135度
        {0.0, 0.0, 1.0, 0.0, 180.0},                  // 180度
        {0.0, 0.0, -0.7071068, 0.7071068, 270.0}      // 270度 (或-90度)
    };

    for (size_t i = 0; i < test_cases.size(); ++i) {
        geometry_msgs::msg::Quaternion quat;
        quat.x = std::get<0>(test_cases[i]);
        quat.y = std::get<1>(test_cases[i]);
        quat.z = std::get<2>(test_cases[i]);
        quat.w = std::get<3>(test_cases[i]);

        double expected = std::get<4>(test_cases[i]);
        double actual = PathConverter::quaternion_to_yaw_degrees(quat);

        std::cout << std::fixed << std::setprecision(1);
        std::cout << "测试 " << (i+1) << ": ";
        std::cout << "四元数(" << quat.x << "," << quat.y << "," << quat.z << "," << quat.w << ") ";
        std::cout << "期望=" << expected << "° ";
        std::cout << "实际=" << actual << "° ";

        double error = std::abs(actual - expected);
        if (error > 180.0) error = 360.0 - error; // 处理环形差异

        if (error < 1.0) {
            std::cout << "✅" << std::endl;
        } else {
            std::cout << "❌ 误差=" << error << "°" << std::endl;
        }
    }
}

/**
 * @brief 详细分析真实路径数据
 */
void analyze_real_path(const nav_msgs::msg::Path& path) {
    std::cout << "🔍 分析真实Nav2路径数据" << std::endl;
    std::cout << "=========================" << std::endl;

    std::cout << "路径信息:" << std::endl;
    std::cout << "  坐标系: " << path.header.frame_id << std::endl;
    std::cout << "  时间戳: " << path.header.stamp.sec << "."
              << std::setfill('0') << std::setw(9) << path.header.stamp.nanosec << std::endl;
    std::cout << "  路径点数: " << path.poses.size() << std::endl;

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\n路径点详情:" << std::endl;
    std::cout << "序号  位置(x,y,z)           四元数(x,y,z,w)                   角度(度)" << std::endl;
    std::cout << "----------------------------------------------------------------" << std::endl;

    for (size_t i = 0; i < path.poses.size(); ++i) {
        const auto& pose = path.poses[i].pose;
        double yaw = PathConverter::quaternion_to_yaw_degrees(pose.orientation);

        std::cout << std::setw(2) << (i+1) << "   ";
        std::cout << "(" << std::setw(5) << pose.position.x
                  << "," << std::setw(5) << pose.position.y
                  << "," << std::setw(5) << pose.position.z << ")  ";
        std::cout << "(" << std::setw(8) << pose.orientation.x
                  << "," << std::setw(8) << pose.orientation.y
                  << "," << std::setw(8) << pose.orientation.z
                  << "," << std::setw(8) << pose.orientation.w << ")  ";
        std::cout << std::setw(6) << yaw << "°" << std::endl;
    }

    // 计算路径总长度
    double total_length = 0.0;
    for (size_t i = 1; i < path.poses.size(); ++i) {
        double dx = path.poses[i].pose.position.x - path.poses[i-1].pose.position.x;
        double dy = path.poses[i].pose.position.y - path.poses[i-1].pose.position.y;
        total_length += std::sqrt(dx*dx + dy*dy);
    }
    std::cout << "\n路径总长度: " << total_length << " 米" << std::endl;
}

/**
 * @brief 测试路径转换功能
 */
void test_path_conversion() {
    std::cout << "\n🛤️  测试ROS2路径到中力协议轨迹转换" << std::endl;
    std::cout << "======================================" << std::endl;

    // 创建路径转换器
    std::string robot_id = "test-robot-001";
    PathConverter converter(robot_id, 0.5, 1.5); // 采样间距0.5米，最大速度1.5m/s

    // 创建真实的Nav2路径
    auto real_path = create_real_nav2_path();

    try {
        // 转换路径
        auto trajectory = converter.convert_path_to_trajectory(real_path);

        std::cout << "转换成功！" << std::endl;
        std::cout << "轨迹信息:" << std::endl;
        std::cout << "  轨迹ID: " << trajectory.trajectoryId << std::endl;
        std::cout << "  时间戳: " << trajectory.timestamp << std::endl;
        std::cout << "  最大速度: " << trajectory.maxSpeed << " m/s" << std::endl;
        std::cout << "  原始点数: " << real_path.poses.size() << std::endl;
        std::cout << "  采样点数: " << trajectory.trajectoryPoints.size() << std::endl;

        double compression_ratio = 100.0 * (1.0 - (double)trajectory.trajectoryPoints.size() / real_path.poses.size());
        std::cout << "  压缩率: " << std::fixed << std::setprecision(1) << compression_ratio << "%" << std::endl;

        std::cout << "\n轨迹点详情:" << std::endl;
        std::cout << "序号  位置(x,y)     角度(度)" << std::endl;
        std::cout << "------------------------" << std::endl;
        for (size_t i = 0; i < trajectory.trajectoryPoints.size(); ++i) {
            const auto& point = trajectory.trajectoryPoints[i];
            std::cout << std::setw(2) << (i+1) << "   ";
            std::cout << "(" << std::setw(5) << std::setprecision(1) << point.x
                      << "," << std::setw(5) << point.y << ")  ";
            std::cout << std::setw(6) << std::setprecision(1) << point.theta << "°" << std::endl;
        }

        // 输出JSON格式（用于MQTT传输）
        std::cout << "\n📄 生成的中力协议轨迹消息 (JSON):" << std::endl;
        std::cout << trajectory.to_json_string() << std::endl;

        // 验证轨迹有效性
        if (converter.validate_trajectory(trajectory.trajectoryPoints)) {
            std::cout << "✅ 轨迹验证通过" << std::endl;
        } else {
            std::cout << "❌ 轨迹验证失败" << std::endl;
        }

        // 显示转换统计
        std::cout << "\n📊 " << converter.get_conversion_stats() << std::endl;

    } catch (const std::exception& e) {
        std::cout << "❌ 转换失败: " << e.what() << std::endl;
    }
}

int main() {
    std::cout << "🧪 真实Nav2路径转换测试" << std::endl;
    std::cout << "========================" << std::endl;

    // 初始化ROS2（用于消息类型）
    rclcpp::init(0, nullptr);

    try {
        // 1. 测试四元数转换精确性
        test_quaternion_conversion();

        // 2. 创建并分析真实路径
        auto real_path = create_real_nav2_path();
        analyze_real_path(real_path);

        // 3. 测试路径转换
        test_path_conversion();

        std::cout << "\n🎉 所有测试完成！" << std::endl;

    } catch (const std::exception& e) {
        std::cout << "❌ 测试失败: " << e.what() << std::endl;
        return -1;
    }

    rclcpp::shutdown();
    return 0;
}