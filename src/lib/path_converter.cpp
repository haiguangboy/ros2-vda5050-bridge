#include "path_converter.hpp"
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <vector>
#include <iostream>

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

    // 如果有动作类型，创建 TrajectoryAction
    std::optional<zhongli_protocol::TrajectoryAction> trajectory_action;
    if (!beta3_info.action_type.empty() && beta3_info.action_type != "none") {
        zhongli_protocol::TrajectoryAction action;
        action.actionType = beta3_info.action_type;

        // 如果有容器类型，添加容器信息
        if (!beta3_info.container_type.empty()) {
            action.containerType = beta3_info.container_type;

            // 创建 ContainerPose
            zhongli_protocol::ContainerPose container_pose;
            container_pose.x = beta3_info.container_x;
            container_pose.y = beta3_info.container_y;
            container_pose.z = beta3_info.container_z;
            container_pose.theta = beta3_info.container_theta;
            container_pose.width = beta3_info.container_width;

            action.containerPose = container_pose;
        }

        trajectory_action = action;
    }

    for (size_t i = 0; i < ros_path.poses.size(); ++i) {
        const auto& pose_stamped = ros_path.poses[i];
        auto point = convert_pose_to_trajectory_point(pose_stamped);
        // 应用beta-3信息
        point.orientation = beta3_info.orientation;
        point.flag = beta3_info.flag;

        // 🔧 修复：只在最后一个点添加动作信息（取货/卸货动作）
        bool is_last_point = (i == ros_path.poses.size() - 1);
        if (trajectory_action.has_value() && is_last_point) {
            point.action = trajectory_action;
        }

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
    info.container_x = 0.0;
    info.container_y = 0.0;
    info.container_z = 0.0;
    info.container_theta = 0.0;
    info.container_width = 1.2;  // 默认容器宽度1.2米

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

            // 解析 orientation 并验证是否符合协议规范
            double parsed_orientation = std::stod(parts[3]);
            // Beta-3协议规范：0（前向）, -3.14（倒车）, 3.14（倒车）
            if (std::abs(parsed_orientation - 0.0) < 0.01) {
                info.orientation = 0.0;  // 前向
            } else if (std::abs(parsed_orientation - 3.14) < 0.01) {
                info.orientation = 3.14;  // 倒车
            } else if (std::abs(parsed_orientation + 3.14) < 0.01) {
                info.orientation = -3.14;  // 倒车
            } else {
                // 非法值，使用默认值并记录警告
                info.orientation = 0.0;
                std::cerr << "警告: orientation值 " << parsed_orientation
                         << " 不符合Beta-3协议规范（应为0, 3.14或-3.14），使用默认值0.0" << std::endl;
            }

            // 解析 flag 并验证是否符合协议规范
            double parsed_flag = std::stod(parts[4]);
            // Beta-3协议规范：只能是0（非分支）或1（进入分支）
            if (std::abs(parsed_flag - 0.0) < 0.01) {
                info.flag = 0.0;  // 非分支
            } else if (std::abs(parsed_flag - 1.0) < 0.01) {
                info.flag = 1.0;  // 进入分支
            } else {
                // 非法值，使用默认值并记录警告
                info.flag = 0.0;
                std::cerr << "警告: flag值 " << parsed_flag
                         << " 不符合Beta-3协议规范（只能是0或1），使用默认值0" << std::endl;
            }

            // 验证 action_type 是否符合协议规范
            if (!info.action_type.empty() && info.action_type != "none") {
                auto action_enum = zhongli_protocol::string_to_action_type(info.action_type);
                if (!action_enum.has_value()) {
                    std::cerr << "警告: action_type值 '" << info.action_type
                             << "' 不符合Beta-3协议规范，有效值为: ground_pick, ground_place, "
                             << "load, unload, pub_load_params, pub_unload_params, start_stacking"
                             << std::endl;
                }
            }

            // 如果有扩展的容器位姿信息，解析它
            if (parts.size() >= 10) {
                info.container_x = std::stod(parts[5]);
                info.container_y = std::stod(parts[6]);
                info.container_z = std::stod(parts[7]);
                info.container_theta = std::stod(parts[8]);
                info.container_width = std::stod(parts[9]);
            }
        } catch (const std::exception& e) {
            // 解析失败，使用默认值
            std::cerr << "解析frame_id失败: " << e.what() << "，使用默认值" << std::endl;
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