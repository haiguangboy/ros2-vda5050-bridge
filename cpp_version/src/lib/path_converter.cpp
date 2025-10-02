#include "path_converter.hpp"
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <vector>

namespace zhongli_bridge {

PathConverter::PathConverter(const std::string& robot_id,
                           double default_speed)
    : robot_id_(robot_id)
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

    // 解析frame_id中的beta-3信息
    auto beta3_info = parse_beta3_info_from_frame_id(ros_path.header.frame_id);

    // 转换所有路径点
    std::vector<zhongli_protocol::TrajectoryPoint> trajectory_points;
    trajectory_points.reserve(ros_path.poses.size());

    for (const auto& pose_stamped : ros_path.poses) {
        auto point = convert_pose_to_trajectory_point(pose_stamped);
        // 应用beta-3信息
        point.orientation = beta3_info.orientation;
        point.flag = beta3_info.flag;
        trajectory_points.push_back(point);
    }

    // 验证轨迹有效性
    if (!validate_trajectory(trajectory_points)) {
        throw std::runtime_error("Generated trajectory is invalid");
    }

    // 更新统计信息
    update_stats(trajectory_points.size(), trajectory_points.size());

    // 创建轨迹消息
    zhongli_protocol::TrajectoryMessage trajectory;
    trajectory.timestamp = zhongli_protocol::create_timestamp();
    trajectory.trajectoryId = zhongli_protocol::generate_trajectory_id(robot_id_);
    trajectory.trajectoryPoints = trajectory_points;
    trajectory.maxSpeed = default_speed_;

    return trajectory;
}

zhongli_protocol::TrajectoryPoint PathConverter::convert_pose_to_trajectory_point(
    const geometry_msgs::msg::PoseStamped& pose_stamped) {

    zhongli_protocol::TrajectoryPoint point;
    point.x = pose_stamped.pose.position.x;
    point.y = pose_stamped.pose.position.y;
    point.theta = quaternion_to_yaw_radians(pose_stamped.pose.orientation);
    point.orientation = 0.0;  // 默认前向运动
    point.flag = 0.0;         // 默认非进入分支

    return point;
}

zhongli_protocol::TrajectoryPoint PathConverter::convert_pose_to_trajectory_point_with_action(
    const geometry_msgs::msg::PoseStamped& pose_stamped,
    const std::optional<zhongli_protocol::TrajectoryAction>& action) {

    zhongli_protocol::TrajectoryPoint point;
    point.x = pose_stamped.pose.position.x;
    point.y = pose_stamped.pose.position.y;
    point.theta = quaternion_to_yaw_radians(pose_stamped.pose.orientation);
    point.orientation = 0.0;  // 默认前向运动
    point.flag = 0.0;         // 默认非进入分支
    point.action = action;

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

PathConverter::Beta3Info PathConverter::parse_beta3_info_from_frame_id(const std::string& frame_id) {
    Beta3Info info;
    info.orientation = 0.0;  // 默认前向
    info.flag = 0.0;         // 默认非分支
    info.action_type = "";
    info.container_type = "";

    // 解析格式: "map|action_type|container_type|orientation|flag|container_x|container_y|container_z|container_theta|container_width"
    std::vector<std::string> parts;
    std::stringstream ss(frame_id);
    std::string item;

    while (std::getline(ss, item, '|')) {
        parts.push_back(item);
    }

    if (parts.size() >= 5) {
        try {
            info.action_type = parts[1];
            info.container_type = parts[2];
            info.orientation = std::stod(parts[3]);
            info.flag = std::stod(parts[4]);

            // 如果有扩展的容器位姿信息，可以在这里解析
            // if (parts.size() >= 10) {
            //     double container_x = std::stod(parts[5]);
            //     double container_y = std::stod(parts[6]);
            //     double container_z = std::stod(parts[7]);
            //     double container_theta = std::stod(parts[8]);
            //     double container_width = std::stod(parts[9]);
            //     // 可以创建动作信息并赋给轨迹点
            // }
        } catch (const std::exception& e) {
            // 解析失败，使用默认值
        }
    }

    return info;
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