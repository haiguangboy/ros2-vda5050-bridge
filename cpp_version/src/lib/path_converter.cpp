#include "path_converter.hpp"
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>

namespace zhongli_bridge {

PathConverter::PathConverter(const std::string& robot_id,
                           double sampling_distance,
                           double default_speed)
    : robot_id_(robot_id)
    , sampling_distance_(sampling_distance)
    , default_speed_(default_speed)
    , total_conversions_(0)
    , total_original_points_(0)
    , total_sampled_points_(0) {
}

zhongli_protocol::TrajectoryMessage PathConverter::convert_path_to_trajectory(
    const nav_msgs::msg::Path& ros_path) {

    if (path_utils::is_path_empty(ros_path)) {
        throw std::invalid_argument("Cannot convert empty path to trajectory");
    }

    // 转换所有路径点
    std::vector<zhongli_protocol::TrajectoryPoint> trajectory_points;
    trajectory_points.reserve(ros_path.poses.size());

    for (const auto& pose_stamped : ros_path.poses) {
        trajectory_points.push_back(convert_pose_to_trajectory_point(pose_stamped));
    }

    // 路径采样优化
    std::vector<zhongli_protocol::TrajectoryPoint> sampled_points =
        sample_trajectory_points(trajectory_points);

    // 验证轨迹有效性
    if (!validate_trajectory(sampled_points)) {
        throw std::runtime_error("Generated trajectory is invalid");
    }

    // 更新统计信息
    update_stats(trajectory_points.size(), sampled_points.size());

    // 创建轨迹消息
    zhongli_protocol::TrajectoryMessage trajectory;
    trajectory.timestamp = zhongli_protocol::create_timestamp();
    trajectory.trajectoryId = zhongli_protocol::generate_trajectory_id(robot_id_);
    trajectory.trajectoryPoints = sampled_points;
    trajectory.maxSpeed = default_speed_;

    return trajectory;
}

zhongli_protocol::TrajectoryPoint PathConverter::convert_pose_to_trajectory_point(
    const geometry_msgs::msg::PoseStamped& pose_stamped) {

    zhongli_protocol::TrajectoryPoint point;
    point.x = pose_stamped.pose.position.x;
    point.y = pose_stamped.pose.position.y;
    point.theta = quaternion_to_yaw_radians(pose_stamped.pose.orientation);

    return point;
}

double PathConverter::quaternion_to_yaw_radians(
    const geometry_msgs::msg::Quaternion& quaternion) {

    // 使用tf2进行四元数到欧拉角转换
    tf2::Quaternion tf_quaternion;
    tf2::fromMsg(quaternion, tf_quaternion);

    // 计算偏航角（弧度）
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);

    // 确保角度在-π到+π弧度范围内
    while (yaw < -M_PI) yaw += 2.0 * M_PI;
    while (yaw >= M_PI) yaw -= 2.0 * M_PI;

    return yaw;
}

std::vector<zhongli_protocol::TrajectoryPoint> PathConverter::sample_trajectory_points(
    const std::vector<zhongli_protocol::TrajectoryPoint>& original_points) {

    if (original_points.empty()) {
        return {};
    }

    if (original_points.size() == 1) {
        return original_points;
    }

    std::vector<zhongli_protocol::TrajectoryPoint> sampled_points;
    sampled_points.push_back(original_points[0]); // 总是包含起点

    double accumulated_distance = 0.0;

    for (size_t i = 1; i < original_points.size(); ++i) {
        double distance = calculate_distance(original_points[i-1], original_points[i]);
        accumulated_distance += distance;

        // 如果累积距离达到采样间距，添加当前点
        if (accumulated_distance >= sampling_distance_) {
            sampled_points.push_back(original_points[i]);
            accumulated_distance = 0.0;
        }
    }

    // 总是包含终点（如果不是最后添加的点）
    if (sampled_points.size() < 2 ||
        calculate_distance(sampled_points.back(), original_points.back()) > 0.01) {
        sampled_points.push_back(original_points.back());
    }

    return sampled_points;
}

double PathConverter::calculate_distance(
    const zhongli_protocol::TrajectoryPoint& p1,
    const zhongli_protocol::TrajectoryPoint& p2) {

    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    return std::sqrt(dx * dx + dy * dy);
}

bool PathConverter::validate_trajectory(
    const std::vector<zhongli_protocol::TrajectoryPoint>& trajectory_points) {

    // 基本验证
    if (trajectory_points.empty()) {
        return false;
    }

    // 检查每个点的有效性
    for (const auto& point : trajectory_points) {
        // 检查坐标是否为有效数值
        if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.theta)) {
            return false;
        }

        // 检查坐标范围（假设合理的工作区域）
        if (std::abs(point.x) > 1000.0 || std::abs(point.y) > 1000.0) {
            return false;
        }

        // 检查角度范围（弧度制：-π到+π）
        if (point.theta < -M_PI || point.theta > M_PI) {
            return false;
        }
    }

    // 检查轨迹连续性（相邻点距离不应过大）
    const double MAX_SEGMENT_LENGTH = 10.0; // 米
    for (size_t i = 1; i < trajectory_points.size(); ++i) {
        double distance = calculate_distance(trajectory_points[i-1], trajectory_points[i]);
        if (distance > MAX_SEGMENT_LENGTH) {
            return false;
        }
    }

    return true;
}

std::string PathConverter::get_conversion_stats() const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(1);

    ss << "Path Conversion Statistics:" << std::endl;
    ss << "  Total conversions: " << total_conversions_ << std::endl;
    ss << "  Average original points: "
       << (total_conversions_ > 0 ? (double)total_original_points_ / total_conversions_ : 0.0)
       << std::endl;
    ss << "  Average sampled points: "
       << (total_conversions_ > 0 ? (double)total_sampled_points_ / total_conversions_ : 0.0)
       << std::endl;
    ss << "  Compression ratio: "
       << (total_original_points_ > 0 ?
           100.0 * (1.0 - (double)total_sampled_points_ / total_original_points_) : 0.0)
       << "%" << std::endl;

    return ss.str();
}

void PathConverter::update_stats(size_t original_count, size_t sampled_count) const {
    ++total_conversions_;
    total_original_points_ += original_count;
    total_sampled_points_ += sampled_count;
}

// 路径工具函数实现
namespace path_utils {

bool is_path_empty(const nav_msgs::msg::Path& path) {
    return path.poses.empty();
}

double calculate_path_length(const nav_msgs::msg::Path& path) {
    if (path.poses.size() < 2) {
        return 0.0;
    }

    double total_length = 0.0;
    for (size_t i = 1; i < path.poses.size(); ++i) {
        double dx = path.poses[i].pose.position.x - path.poses[i-1].pose.position.x;
        double dy = path.poses[i].pose.position.y - path.poses[i-1].pose.position.y;
        total_length += std::sqrt(dx * dx + dy * dy);
    }

    return total_length;
}

std::pair<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::PoseStamped>
get_path_endpoints(const nav_msgs::msg::Path& path) {
    if (path.poses.empty()) {
        throw std::invalid_argument("Cannot get endpoints of empty path");
    }

    return std::make_pair(path.poses.front(), path.poses.back());
}

} // namespace path_utils

} // namespace zhongli_bridge