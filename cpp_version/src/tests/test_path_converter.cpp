#include <gtest/gtest.h>
#include "path_converter.hpp"
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace zhongli_bridge;

class PathConverterTest : public ::testing::Test {
protected:
    void SetUp() override {
        robot_id_ = "test-robot-001";
        converter_ = std::make_unique<PathConverter>(robot_id_, 0.5, 1.5);
    }

    nav_msgs::msg::Path create_test_path() {
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        path.header.stamp = rclcpp::Clock().now();

        // 创建测试路径点
        std::vector<std::pair<double, double>> waypoints = {
            {0.0, 0.0}, {2.0, 0.0}, {4.0, 1.0}, {6.0, 3.0}, {8.0, 3.0}
        };

        for (size_t i = 0; i < waypoints.size(); ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = waypoints[i].first;
            pose.pose.position.y = waypoints[i].second;
            pose.pose.position.z = 0.0;

            // 计算朝向下一个点的方向
            if (i < waypoints.size() - 1) {
                double dx = waypoints[i+1].first - waypoints[i].first;
                double dy = waypoints[i+1].second - waypoints[i].second;
                double yaw = std::atan2(dy, dx);

                tf2::Quaternion q;
                q.setRPY(0, 0, yaw);
                pose.pose.orientation = tf2::toMsg(q);
            } else {
                // 最后一个点保持前一个点的方向
                pose.pose.orientation = path.poses.back().pose.orientation;
            }

            path.poses.push_back(pose);
        }

        return path;
    }

    std::string robot_id_;
    std::unique_ptr<PathConverter> converter_;
};

// 测试基础路径转换
TEST_F(PathConverterTest, TestBasicPathConversion) {
    auto test_path = create_test_path();

    // 转换路径
    auto trajectory = converter_->convert_path_to_trajectory(test_path);

    // 验证基本属性
    EXPECT_FALSE(trajectory.timestamp.empty());
    EXPECT_FALSE(trajectory.trajectoryId.empty());
    EXPECT_TRUE(trajectory.trajectoryId.find(robot_id_) != std::string::npos);
    EXPECT_DOUBLE_EQ(trajectory.maxSpeed, 1.5);
    EXPECT_FALSE(trajectory.trajectoryPoints.empty());

    // 验证轨迹点数量（应该经过采样优化）
    EXPECT_LE(trajectory.trajectoryPoints.size(), test_path.poses.size());

    // 验证第一个和最后一个点
    auto& first_point = trajectory.trajectoryPoints.front();
    auto& last_point = trajectory.trajectoryPoints.back();

    EXPECT_DOUBLE_EQ(first_point.x, 0.0);
    EXPECT_DOUBLE_EQ(first_point.y, 0.0);
    EXPECT_DOUBLE_EQ(last_point.x, 8.0);
    EXPECT_DOUBLE_EQ(last_point.y, 3.0);
}

// 测试空路径处理
TEST_F(PathConverterTest, TestEmptyPathHandling) {
    nav_msgs::msg::Path empty_path;

    EXPECT_THROW(
        converter_->convert_path_to_trajectory(empty_path),
        std::invalid_argument
    );
}

// 测试四元数到角度转换
TEST_F(PathConverterTest, TestQuaternionToYawConversion) {
    // 测试0度
    tf2::Quaternion q0;
    q0.setRPY(0, 0, 0);
    geometry_msgs::msg::Quaternion quat0 = tf2::toMsg(q0);
    EXPECT_NEAR(PathConverter::quaternion_to_yaw_degrees(quat0), 0.0, 0.1);

    // 测试90度
    tf2::Quaternion q90;
    q90.setRPY(0, 0, M_PI/2);
    geometry_msgs::msg::Quaternion quat90 = tf2::toMsg(q90);
    EXPECT_NEAR(PathConverter::quaternion_to_yaw_degrees(quat90), 90.0, 0.1);

    // 测试180度
    tf2::Quaternion q180;
    q180.setRPY(0, 0, M_PI);
    geometry_msgs::msg::Quaternion quat180 = tf2::toMsg(q180);
    EXPECT_NEAR(PathConverter::quaternion_to_yaw_degrees(quat180), 180.0, 0.1);

    // 测试270度
    tf2::Quaternion q270;
    q270.setRPY(0, 0, -M_PI/2);
    geometry_msgs::msg::Quaternion quat270 = tf2::toMsg(q270);
    EXPECT_NEAR(PathConverter::quaternion_to_yaw_degrees(quat270), 270.0, 0.1);
}

// 测试轨迹点采样
TEST_F(PathConverterTest, TestTrajectorySampling) {
    // 创建密集点集
    std::vector<zhongli_protocol::TrajectoryPoint> dense_points;
    for (int i = 0; i <= 100; ++i) {
        dense_points.push_back({i * 0.1, 0.0, 0.0});
    }

    // 设置采样间距为0.5米
    converter_->set_sampling_distance(0.5);
    auto sampled_points = converter_->sample_trajectory_points(dense_points);

    // 验证采样效果
    EXPECT_LT(sampled_points.size(), dense_points.size());
    EXPECT_GE(sampled_points.size(), 2); // 至少包含起点和终点

    // 验证起点和终点保持不变
    EXPECT_DOUBLE_EQ(sampled_points.front().x, 0.0);
    EXPECT_DOUBLE_EQ(sampled_points.back().x, 10.0);
}

// 测试距离计算
TEST_F(PathConverterTest, TestDistanceCalculation) {
    zhongli_protocol::TrajectoryPoint p1{0.0, 0.0, 0.0};
    zhongli_protocol::TrajectoryPoint p2{3.0, 4.0, 0.0};

    double distance = PathConverter::calculate_distance(p1, p2);
    EXPECT_DOUBLE_EQ(distance, 5.0); // 3-4-5直角三角形
}

// 测试轨迹验证
TEST_F(PathConverterTest, TestTrajectoryValidation) {
    // 有效轨迹
    std::vector<zhongli_protocol::TrajectoryPoint> valid_trajectory = {
        {0.0, 0.0, 0.0},
        {1.0, 1.0, 45.0},
        {2.0, 2.0, 90.0}
    };
    EXPECT_TRUE(converter_->validate_trajectory(valid_trajectory));

    // 空轨迹
    std::vector<zhongli_protocol::TrajectoryPoint> empty_trajectory;
    EXPECT_FALSE(converter_->validate_trajectory(empty_trajectory));

    // 包含NaN的轨迹
    std::vector<zhongli_protocol::TrajectoryPoint> nan_trajectory = {
        {0.0, 0.0, 0.0},
        {std::numeric_limits<double>::quiet_NaN(), 1.0, 45.0}
    };
    EXPECT_FALSE(converter_->validate_trajectory(nan_trajectory));

    // 角度超出范围的轨迹
    std::vector<zhongli_protocol::TrajectoryPoint> invalid_angle_trajectory = {
        {0.0, 0.0, 0.0},
        {1.0, 1.0, 400.0} // 超出360度
    };
    EXPECT_FALSE(converter_->validate_trajectory(invalid_angle_trajectory));

    // 距离过大的轨迹
    std::vector<zhongli_protocol::TrajectoryPoint> large_distance_trajectory = {
        {0.0, 0.0, 0.0},
        {100.0, 100.0, 45.0} // 距离约141米，超过10米限制
    };
    EXPECT_FALSE(converter_->validate_trajectory(large_distance_trajectory));
}

// 测试路径工具函数
TEST_F(PathConverterTest, TestPathUtils) {
    // 测试空路径检测
    nav_msgs::msg::Path empty_path;
    EXPECT_TRUE(path_utils::is_path_empty(empty_path));

    auto test_path = create_test_path();
    EXPECT_FALSE(path_utils::is_path_empty(test_path));

    // 测试路径长度计算
    double length = path_utils::calculate_path_length(test_path);
    EXPECT_GT(length, 0.0);

    // 测试端点获取
    auto endpoints = path_utils::get_path_endpoints(test_path);
    EXPECT_DOUBLE_EQ(endpoints.first.pose.position.x, 0.0);
    EXPECT_DOUBLE_EQ(endpoints.second.pose.position.x, 8.0);
}

// 测试统计信息
TEST_F(PathConverterTest, TestConversionStats) {
    auto test_path = create_test_path();

    // 执行几次转换
    for (int i = 0; i < 3; ++i) {
        converter_->convert_path_to_trajectory(test_path);
    }

    // 获取统计信息
    std::string stats = converter_->get_conversion_stats();
    EXPECT_FALSE(stats.empty());
    EXPECT_TRUE(stats.find("Total conversions: 3") != std::string::npos);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);

    auto result = RUN_ALL_TESTS();

    rclcpp::shutdown();
    return result;
}