#pragma once

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <string>
#include "zhongli_protocol_types.hpp"

namespace zhongli_bridge {

/**
 * @brief ROS2路径与中力协议轨迹转换器
 *
 * 实现ROS2 Nav2路径到中力具身机器人协议轨迹的高效转换
 */
class PathConverter {
public:
    /**
     * @brief 构造函数
     *
     * @param robot_id 机器人ID
     * @param default_speed 默认最大速度（米/秒）
     */
    PathConverter(const std::string& robot_id,
                  double default_speed = 1.5);

    /**
     * @brief 将ROS2路径转换为中力协议轨迹消息
     *
     * @param ros_path ROS2路径消息
     * @return zhongli_protocol::TrajectoryMessage 轨迹消息
     */
    zhongli_protocol::TrajectoryMessage convert_path_to_trajectory(
        const nav_msgs::msg::Path& ros_path);

    /**
     * @brief 将ROS2姿态转换为中力协议轨迹点
     *
     * @param pose_stamped ROS2姿态
     * @return zhongli_protocol::TrajectoryPoint 轨迹点
     */
    zhongli_protocol::TrajectoryPoint convert_pose_to_trajectory_point(
        const geometry_msgs::msg::PoseStamped& pose_stamped);

    /**
     * @brief 四元数转换为欧拉角（弧度）
     *
     * @param quaternion 四元数
     * @return double 偏航角（弧度，范围：-π到+π）
     */
    static double quaternion_to_yaw_radians(const geometry_msgs::msg::Quaternion& quaternion);


    /**
     * @brief 计算两点间距离
     *
     * @param p1 点1
     * @param p2 点2
     * @return double 距离（米）
     */
    static double calculate_distance(const zhongli_protocol::TrajectoryPoint& p1,
                                   const zhongli_protocol::TrajectoryPoint& p2);

    /**
     * @brief 验证轨迹有效性
     *
     * @param trajectory_points 轨迹点列表
     * @return true 轨迹有效
     * @return false 轨迹无效
     */
    bool validate_trajectory(const std::vector<zhongli_protocol::TrajectoryPoint>& trajectory_points);


    /**
     * @brief 设置默认速度
     *
     * @param speed 默认速度（米/秒）
     */
    void set_default_speed(double speed) { default_speed_ = speed; }

    /**
     * @brief 获取转换统计信息
     *
     * @return std::string 统计信息字符串
     */
    std::string get_conversion_stats() const;

private:
    std::string robot_id_;
    double default_speed_;      ///< 默认最大速度

    // 统计信息
    mutable size_t total_conversions_;
    mutable size_t total_original_points_;
    mutable size_t total_sampled_points_;

    /**
     * @brief 更新统计信息
     *
     * @param original_count 原始点数
     * @param sampled_count 采样点数
     */
    void update_stats(size_t original_count, size_t sampled_count) const;
};

/**
 * @brief 路径转换工具函数
 */
namespace path_utils {

    /**
     * @brief 检查路径是否为空
     *
     * @param path ROS2路径
     * @return true 路径为空
     * @return false 路径不为空
     */
    bool is_path_empty(const nav_msgs::msg::Path& path);

    /**
     * @brief 计算路径总长度
     *
     * @param path ROS2路径
     * @return double 路径长度（米）
     */
    double calculate_path_length(const nav_msgs::msg::Path& path);

    /**
     * @brief 获取路径起点和终点
     *
     * @param path ROS2路径
     * @return std::pair<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::PoseStamped>
     *         起点和终点
     */
    std::pair<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::PoseStamped>
    get_path_endpoints(const nav_msgs::msg::Path& path);

} // namespace path_utils

} // namespace zhongli_bridge