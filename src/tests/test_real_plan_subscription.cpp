#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "path_converter.hpp"
#include "zhongli_protocol_types.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>

using namespace zhongli_bridge;

/**
 * @brief 测试节点：订阅真实的/plan话题并测试转换
 */
class RealPlanSubscriptionTest : public rclcpp::Node {
public:
    RealPlanSubscriptionTest() : Node("real_plan_test_node") {
        std::cout << "🧪 真实/plan话题订阅测试节点启动" << std::endl;
        std::cout << "====================================" << std::endl;

        // 创建路径转换器
        robot_id_ = "test-robot-001";
        path_converter_ = std::make_unique<PathConverter>(robot_id_, 1.5);

        // 订阅真实的/plan话题
        path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan", 10,
            std::bind(&RealPlanSubscriptionTest::path_callback, this, std::placeholders::_1));

        std::cout << "✅ 已订阅/plan话题，等待Nav2发布路径数据..." << std::endl;
        std::cout << "💡 确保test_nav_path_publisher.py正在运行" << std::endl;
        std::cout << "按Ctrl+C退出测试" << std::endl;
        std::cout << "====================================" << std::endl;

        // 记录测试开始时间
        test_start_time_ = std::chrono::steady_clock::now();
        test_count_ = 0;
    }

private:
    /**
     * @brief 路径消息回调 - 处理真实的Nav2路径数据
     */
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        test_count_++;
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - test_start_time_).count();

        std::cout << "\n📍 [测试 " << test_count_ << "] 收到真实/plan话题数据 (+" << elapsed << "ms)" << std::endl;
        std::cout << "================================================" << std::endl;

        try {
            // 1. 分析接收到的路径数据
            analyze_received_path(*msg);

            // 2. 测试路径转换
            test_path_conversion(*msg);

            // 3. 显示转换结果
            std::cout << "✅ 测试 " << test_count_ << " 完成" << std::endl;

        } catch (const std::exception& e) {
            std::cout << "❌ 测试 " << test_count_ << " 失败: " << e.what() << std::endl;
        }

        std::cout << "================================================" << std::endl;
    }

    /**
     * @brief 分析接收到的真实路径数据
     */
    void analyze_received_path(const nav_msgs::msg::Path& path) {
        std::cout << "🔍 路径数据分析:" << std::endl;
        std::cout << "  坐标系: " << path.header.frame_id << std::endl;
        std::cout << "  时间戳: " << path.header.stamp.sec << "."
                  << std::setfill('0') << std::setw(9) << path.header.stamp.nanosec << std::endl;
        std::cout << "  路径点数: " << path.poses.size() << std::endl;

        if (path.poses.empty()) {
            std::cout << "⚠️  警告: 收到空路径！" << std::endl;
            return;
        }

        // 显示前几个路径点的详情
        size_t show_count = std::min(size_t(5), path.poses.size());
        std::cout << "  前" << show_count << "个路径点详情:" << std::endl;

        for (size_t i = 0; i < show_count; ++i) {
            const auto& pose = path.poses[i].pose;
            double yaw = PathConverter::quaternion_to_yaw_radians(pose.orientation);

            std::cout << "    [" << (i+1) << "] 位置("
                      << std::fixed << std::setprecision(2)
                      << pose.position.x << "," << pose.position.y << ") "
                      << "角度=" << std::setprecision(1) << yaw << "°" << std::endl;
        }

        if (path.poses.size() > show_count) {
            std::cout << "    ... (还有" << (path.poses.size() - show_count) << "个点)" << std::endl;
        }

        // 计算路径总长度
        double total_length = 0.0;
        for (size_t i = 1; i < path.poses.size(); ++i) {
            double dx = path.poses[i].pose.position.x - path.poses[i-1].pose.position.x;
            double dy = path.poses[i].pose.position.y - path.poses[i-1].pose.position.y;
            total_length += std::sqrt(dx*dx + dy*dy);
        }
        std::cout << "  路径总长度: " << std::fixed << std::setprecision(3) << total_length << " 米" << std::endl;
    }

    /**
     * @brief 测试路径转换功能
     */
    void test_path_conversion(const nav_msgs::msg::Path& path) {
        std::cout << "\n🛤️  开始路径转换测试:" << std::endl;

        auto start_time = std::chrono::high_resolution_clock::now();

        // 转换路径为中力协议轨迹
        auto trajectory = path_converter_->convert_path_to_trajectory(path);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto conversion_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

        std::cout << "✅ 转换成功！用时: " << conversion_time << " μs" << std::endl;
        std::cout << "  轨迹ID: " << trajectory.trajectoryId << std::endl;
        std::cout << "  最大速度: " << trajectory.maxSpeed << " m/s" << std::endl;
        std::cout << "  原始点数: " << path.poses.size() << std::endl;
        std::cout << "  优化点数: " << trajectory.trajectoryPoints.size() << std::endl;

        double compression_ratio = 100.0 * (1.0 - (double)trajectory.trajectoryPoints.size() / path.poses.size());
        std::cout << "  压缩率: " << std::fixed << std::setprecision(1) << compression_ratio << "%" << std::endl;

        // 显示生成的JSON（前500字符）
        std::string json_str = trajectory.to_json_string();
        std::cout << "\n📄 生成的中力协议JSON (前500字符):" << std::endl;
        if (json_str.length() > 500) {
            std::cout << json_str.substr(0, 500) << "..." << std::endl;
            std::cout << "  (完整JSON长度: " << json_str.length() << " 字符)" << std::endl;
        } else {
            std::cout << json_str << std::endl;
        }

        // 验证轨迹有效性
        if (path_converter_->validate_trajectory(trajectory.trajectoryPoints)) {
            std::cout << "✅ 轨迹验证通过" << std::endl;
        } else {
            std::cout << "❌ 轨迹验证失败" << std::endl;
        }
    }

    // 成员变量
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
    std::unique_ptr<PathConverter> path_converter_;
    std::string robot_id_;
    std::chrono::steady_clock::time_point test_start_time_;
    int test_count_;
};

int main(int argc, char** argv) {
    // 初始化ROS2
    rclcpp::init(argc, argv);

    std::cout << "🚀 真实Nav2 /plan话题订阅测试" << std::endl;
    std::cout << "============================" << std::endl;
    std::cout << "本测试订阅真实的/plan话题，测试C++桥接器的路径处理能力" << std::endl;
    std::cout << "请确保test_nav_path_publisher.py正在后台运行" << std::endl;
    std::cout << "============================" << std::endl;

    try {
        // 创建测试节点
        auto test_node = std::make_shared<RealPlanSubscriptionTest>();

        // 运行节点，等待/plan话题数据
        rclcpp::spin(test_node);

    } catch (const std::exception& e) {
        std::cout << "❌ 测试失败: " << e.what() << std::endl;
        return -1;
    }

    rclcpp::shutdown();
    std::cout << "\n✅ 测试节点正常退出" << std::endl;
    return 0;
}