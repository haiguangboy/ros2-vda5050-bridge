#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "zhongli_protocol_types.hpp"
#include "zhongli_mqtt_client.hpp"
#include "path_converter.hpp"

#include <memory>
#include <string>
#include <chrono>
#include <thread>

namespace zhongli_bridge {

/**
 * @brief ROS2中力协议桥接器主节点
 *
 * 实现ROS2导航系统与中力具身机器人协议的桥接功能：
 * - ROS2路径自动转换为中力协议轨迹
 * - 实时状态反馈和同步
 * - 双向通信和错误处理
 */
class ROS2ZhongliBridge : public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     *
     * @param node_options ROS2节点选项
     */
    explicit ROS2ZhongliBridge(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());

    /**
     * @brief 析构函数
     */
    ~ROS2ZhongliBridge();

    /**
     * @brief 初始化桥接器
     *
     * @return true 初始化成功
     * @return false 初始化失败
     */
    bool initialize();

    /**
     * @brief 启动桥接器服务
     */
    void start();

    /**
     * @brief 停止桥接器服务
     */
    void stop();

    /**
     * @brief 获取桥接器状态
     *
     * @return std::string 状态描述
     */
    std::string get_bridge_status() const;

private:
    // ROS2订阅器
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navigation_result_subscription_;

    // ROS2发布器
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cancel_navigation_publisher_;

    // TF2变换
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    // 核心组件
    std::unique_ptr<zhongli_protocol::ZhongliMqttClient> mqtt_client_;
    std::unique_ptr<PathConverter> path_converter_;

    // 定时器
    rclcpp::TimerBase::SharedPtr state_publish_timer_;

    // 参数
    std::string robot_id_;
    std::string mqtt_broker_host_;
    int mqtt_broker_port_;
    std::string map_frame_;
    std::string base_frame_;
    double state_publish_rate_;
    double goal_tolerance_xy_;
    double goal_tolerance_theta_;

    // 状态变量
    geometry_msgs::msg::PoseWithCovarianceStamped current_pose_;
    geometry_msgs::msg::Twist current_velocity_;
    nav_msgs::msg::OccupancyGrid current_map_;
    bool bridge_running_;
    std::chrono::steady_clock::time_point last_path_time_;

    // 当前执行状态
    std::string current_trajectory_id_;
    std::string current_action_id_;
    std::string current_task_id_;
    bool goal_reached_;

    // 线程管理
    std::unique_ptr<std::thread> state_monitor_thread_;
    std::atomic<bool> should_stop_;
    rclcpp::TimerBase::SharedPtr task_completion_timer_;

    // ROS2回调函数

    /**
     * @brief 路径消息回调
     *
     * @param msg 路径消息
     */
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);

    /**
     * @brief 地图消息回调
     *
     * @param msg 地图消息
     */
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    /**
     * @brief 位姿消息回调
     *
     * @param msg 位姿消息
     */
    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    /**
     * @brief 速度命令回调
     *
     * @param msg 速度命令消息
     */
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    /**
     * @brief 导航结果回调
     *
     * @param msg 导航结果消息
     */
    void navigation_result_callback(const std_msgs::msg::String::SharedPtr msg);

    // MQTT消息回调函数

    /**
     * @brief 任务消息回调
     *
     * @param task_msg 任务消息
     */
    void handle_task_message(const zhongli_protocol::TaskMessage& task_msg);

    /**
     * @brief 轨迹状态回调
     *
     * @param status_msg 轨迹状态消息
     */
    void handle_trajectory_status(const zhongli_protocol::TrajectoryStatusMessage& status_msg);

    /**
     * @brief 动作状态回调
     *
     * @param status_msg 动作状态消息
     */
    void handle_action_status(const zhongli_protocol::ActionStatusMessage& status_msg);

    // 状态发布相关

    /**
     * @brief 定时器回调 - 发布设备状态
     */
    void state_publish_timer_callback();

    /**
     * @brief 创建设备状态消息
     *
     * @return zhongli_protocol::DeviceStateMessage 设备状态消息
     */
    zhongli_protocol::DeviceStateMessage create_device_state_message();

    /**
     * @brief 状态监控线程函数
     */
    void state_monitor_thread_function();

    // 工具函数

    /**
     * @brief 检查目标是否到达
     *
     * @param target_pose 目标位姿
     * @return true 已到达目标
     * @return false 未到达目标
     */
    bool is_goal_reached(const geometry_msgs::msg::PoseStamped& target_pose);

    /**
     * @brief 获取当前位姿（通过TF2）
     *
     * @return std::optional<geometry_msgs::msg::PoseStamped> 当前位姿
     */
    std::optional<geometry_msgs::msg::PoseStamped> get_current_pose();

    /**
     * @brief 参数声明和获取
     */
    void declare_parameters();

    /**
     * @brief 创建ROS2订阅器和发布器
     */
    void create_ros2_interfaces();

    /**
     * @brief 设置MQTT回调函数
     */
    void setup_mqtt_callbacks();

    /**
     * @brief 记录调试信息
     *
     * @param message 调试消息
     */
    void log_debug(const std::string& message);

    /**
     * @brief 记录错误信息
     *
     * @param message 错误消息
     */
    void log_error(const std::string& message);
};

} // namespace zhongli_bridge