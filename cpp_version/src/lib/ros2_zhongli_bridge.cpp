#include "ros2_zhongli_bridge.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

namespace zhongli_bridge {

ROS2ZhongliBridge::ROS2ZhongliBridge(const rclcpp::NodeOptions& node_options)
    : rclcpp::Node("zhongli_bridge", node_options)
    , bridge_running_(false)
    , goal_reached_(false)
    , should_stop_(false) {

    RCLCPP_INFO(this->get_logger(), "🚀 初始化ROS2中力协议桥接器");

    // 声明参数
    declare_parameters();

    // 初始化TF2
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}

ROS2ZhongliBridge::~ROS2ZhongliBridge() {
    stop();
}

bool ROS2ZhongliBridge::initialize() {
    try {
        // 打印详细的配置信息
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "🔧 ============ 配置信息 ============");
        RCLCPP_INFO(this->get_logger(), "📋 机器人ID: %s", robot_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "🌐 MQTT Broker地址: %s", mqtt_broker_host_.c_str());
        RCLCPP_INFO(this->get_logger(), "🌐 MQTT Broker端口: %d", mqtt_broker_port_);
        RCLCPP_INFO(this->get_logger(), "🗺️  地图坐标系: %s", map_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "🤖 机器人坐标系: %s", base_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "📊 状态发布频率: %.1f Hz", state_publish_rate_);
        RCLCPP_INFO(this->get_logger(), "🎯 位置容差: %.2f m", goal_tolerance_xy_);
        RCLCPP_INFO(this->get_logger(), "🎯 角度容差: %.2f rad", goal_tolerance_theta_);
        RCLCPP_INFO(this->get_logger(), "🔧 ================================");
        RCLCPP_INFO(this->get_logger(), "");

        // 创建路径转换器
        path_converter_ = std::make_unique<PathConverter>(robot_id_);

        // 创建MQTT客户端
        mqtt_client_ = std::make_unique<zhongli_protocol::ZhongliMqttClient>(
            robot_id_, mqtt_broker_host_, mqtt_broker_port_);

        // 连接MQTT（允许虚拟客户端失败）
        if (!mqtt_client_->connect()) {
            RCLCPP_WARN(this->get_logger(), "⚠️  MQTT连接失败，继续使用离线模式");
        } else {
            RCLCPP_INFO(this->get_logger(), "✅ MQTT连接成功");
        }

        // 创建ROS2接口
        create_ros2_interfaces();

        // 设置MQTT回调
        setup_mqtt_callbacks();

        // 创建状态发布定时器
        auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / state_publish_rate_));
        state_publish_timer_ = this->create_wall_timer(
            timer_period,
            std::bind(&ROS2ZhongliBridge::state_publish_timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "✅ 桥接器初始化完成");
        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "❌ 初始化失败: %s", e.what());
        return false;
    }
}

void ROS2ZhongliBridge::start() {
    if (bridge_running_) {
        RCLCPP_WARN(this->get_logger(), "⚠️  桥接器已在运行");
        return;
    }

    bridge_running_ = true;
    should_stop_ = false;

    // 启动状态监控线程
    state_monitor_thread_ = std::make_unique<std::thread>(
        &ROS2ZhongliBridge::state_monitor_thread_function, this);

    RCLCPP_INFO(this->get_logger(), "🎯 桥接器已启动");
}

void ROS2ZhongliBridge::stop() {
    if (!bridge_running_) {
        return;
    }

    should_stop_ = true;
    bridge_running_ = false;

    // 等待线程结束
    if (state_monitor_thread_ && state_monitor_thread_->joinable()) {
        state_monitor_thread_->join();
    }

    // 断开MQTT连接
    if (mqtt_client_) {
        mqtt_client_->disconnect();
    }

    RCLCPP_INFO(this->get_logger(), "🛑 桥接器已停止");
}

std::string ROS2ZhongliBridge::get_bridge_status() const {
    std::stringstream ss;
    ss << "Bridge Status:" << std::endl;
    ss << "  Running: " << (bridge_running_ ? "Yes" : "No") << std::endl;
    ss << "  MQTT: " << (mqtt_client_ ? mqtt_client_->get_connection_status() : "Not initialized") << std::endl;
    ss << "  Current Trajectory: " << current_trajectory_id_ << std::endl;
    ss << "  Goal Reached: " << (goal_reached_ ? "Yes" : "No") << std::endl;

    if (path_converter_) {
        ss << std::endl << path_converter_->get_conversion_stats();
    }

    return ss.str();
}

void ROS2ZhongliBridge::declare_parameters() {
    // 声明所有参数并设置默认值
    this->declare_parameter("robot_id", "robot-001");
    this->declare_parameter("mqtt_broker_host", "localhost");
    this->declare_parameter("mqtt_broker_port", 1883);
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("base_frame", "base_link");
    this->declare_parameter("state_publish_rate", 2.0);
    this->declare_parameter("goal_tolerance_xy", 0.2);
    this->declare_parameter("goal_tolerance_theta", 0.1);

    // 获取参数值
    robot_id_ = this->get_parameter("robot_id").as_string();
    mqtt_broker_host_ = this->get_parameter("mqtt_broker_host").as_string();
    mqtt_broker_port_ = this->get_parameter("mqtt_broker_port").as_int();
    map_frame_ = this->get_parameter("map_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    state_publish_rate_ = this->get_parameter("state_publish_rate").as_double();
    goal_tolerance_xy_ = this->get_parameter("goal_tolerance_xy").as_double();
    goal_tolerance_theta_ = this->get_parameter("goal_tolerance_theta").as_double();
}

void ROS2ZhongliBridge::create_ros2_interfaces() {
    using std::placeholders::_1;

    // 创建订阅器
    path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
        "/plan", 10, std::bind(&ROS2ZhongliBridge::path_callback, this, _1));

    map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&ROS2ZhongliBridge::map_callback, this, _1));

    pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10, std::bind(&ROS2ZhongliBridge::pose_callback, this, _1));

    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&ROS2ZhongliBridge::cmd_vel_callback, this, _1));

    navigation_result_subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/navigation_result", 10, std::bind(&ROS2ZhongliBridge::navigation_result_callback, this, _1));

    // 创建发布器
    goal_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    cancel_navigation_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/cancel_navigation", 10);

    RCLCPP_INFO(this->get_logger(), "✅ ROS2接口创建完成");
}

void ROS2ZhongliBridge::setup_mqtt_callbacks() {
    mqtt_client_->set_task_callback(
        std::bind(&ROS2ZhongliBridge::handle_task_message, this, std::placeholders::_1));

    mqtt_client_->set_trajectory_status_callback(
        std::bind(&ROS2ZhongliBridge::handle_trajectory_status, this, std::placeholders::_1));

    mqtt_client_->set_action_status_callback(
        std::bind(&ROS2ZhongliBridge::handle_action_status, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "✅ MQTT回调设置完成");
}

void ROS2ZhongliBridge::path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    try {
        RCLCPP_INFO(this->get_logger(), "📍 收到路径消息，包含 %zu 个点", msg->poses.size());

        // 转换路径为轨迹
        auto trajectory = path_converter_->convert_path_to_trajectory(*msg);

        // 更新轨迹状态
        current_trajectory_id_ = trajectory.trajectoryId;
        last_path_time_ = std::chrono::steady_clock::now();
        goal_reached_ = false;

        // 发布轨迹到MQTT（如果连接可用）
        if (mqtt_client_->is_connected()) {
            if (mqtt_client_->publish_trajectory(trajectory)) {
                RCLCPP_INFO(this->get_logger(), "🛤️  轨迹发布成功: %s", trajectory.trajectoryId.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "❌ 轨迹发布失败");
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "🛤️  轨迹转换完成: %s (离线模式)", trajectory.trajectoryId.c_str());
        }

        log_debug("轨迹包含 " + std::to_string(trajectory.trajectoryPoints.size()) + " 个点");

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "❌ 路径转换错误: %s", e.what());
    }
}

void ROS2ZhongliBridge::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    current_map_ = *msg;
    log_debug("地图更新: " + std::to_string(msg->info.width) + "x" + std::to_string(msg->info.height));
}

void ROS2ZhongliBridge::pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    current_pose_ = *msg;
}

void ROS2ZhongliBridge::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    current_velocity_ = *msg;
}

void ROS2ZhongliBridge::navigation_result_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "🎯 导航结果: %s", msg->data.c_str());

    if (msg->data == "success") {
        goal_reached_ = true;
    }
}

void ROS2ZhongliBridge::handle_task_message(const zhongli_protocol::TaskMessage& task_msg) {
    RCLCPP_INFO(this->get_logger(), "📋 收到任务: %s", task_msg.taskId.c_str());

    current_task_id_ = task_msg.taskId;

    // 这里可以添加任务处理逻辑
    // 例如：根据任务设置导航目标

    log_debug("任务从 " + task_msg.startArea + " 到 " + task_msg.targetArea);
}

void ROS2ZhongliBridge::handle_trajectory_status(const zhongli_protocol::TrajectoryStatusMessage& status_msg) {
    RCLCPP_INFO(this->get_logger(), "📊 轨迹状态更新: %s - %s",
                status_msg.trajectoryId.c_str(), status_msg.status.c_str());

    if (status_msg.status == "completed") {
        goal_reached_ = true;
    } else if (status_msg.status == "failed") {
        log_error("轨迹执行失败: " + status_msg.errorDesc);
    }
}

void ROS2ZhongliBridge::handle_action_status(const zhongli_protocol::ActionStatusMessage& status_msg) {
    RCLCPP_INFO(this->get_logger(), "🎬 动作状态更新: %s - %s",
                status_msg.actionId.c_str(), status_msg.status.c_str());

    if (status_msg.status == "failed") {
        log_error("动作执行失败: " + status_msg.errorDesc);
    }
}

void ROS2ZhongliBridge::state_publish_timer_callback() {
    if (!bridge_running_ || !mqtt_client_->is_connected()) {
        return;
    }

    try {
        auto state_msg = create_device_state_message();
        mqtt_client_->publish_device_state(state_msg);
    } catch (const std::exception& e) {
        log_error("状态发布失败: " + std::string(e.what()));
    }
}

zhongli_protocol::DeviceStateMessage ROS2ZhongliBridge::create_device_state_message() {
    zhongli_protocol::DeviceStateMessage state_msg;

    state_msg.timestamp = zhongli_protocol::create_timestamp();

    // 位姿信息
    state_msg.pose.x = current_pose_.pose.pose.position.x;
    state_msg.pose.y = current_pose_.pose.pose.position.y;
    state_msg.pose.theta = PathConverter::quaternion_to_yaw_degrees(current_pose_.pose.pose.orientation);

    // 货叉状态（模拟数据）
    state_msg.forkliftState.height = 0.0;
    state_msg.forkliftState.weight = 0.0;
    state_msg.forkliftState.lateralShift = 0.0;
    state_msg.forkliftState.forwardExtension = 0.0;
    state_msg.forkliftState.tiltBack = false;
    state_msg.forkliftState.status = "ready";

    // 电池状态（模拟数据）
    state_msg.battery.level = 85;
    state_msg.battery.charging = false;

    // 系统状态
    if (bridge_running_) {
        state_msg.systemState = "running";
    } else {
        state_msg.systemState = "idle";
    }

    // 错误信息（当前为空）
    state_msg.errors.clear();

    return state_msg;
}

void ROS2ZhongliBridge::state_monitor_thread_function() {
    const auto sleep_duration = std::chrono::milliseconds(100);

    while (!should_stop_) {
        try {
            // 监控系统状态
            // 这里可以添加额外的状态检查逻辑

            std::this_thread::sleep_for(sleep_duration);
        } catch (const std::exception& e) {
            log_error("状态监控线程错误: " + std::string(e.what()));
        }
    }
}

bool ROS2ZhongliBridge::is_goal_reached(const geometry_msgs::msg::PoseStamped& target_pose) {
    auto current_pose_opt = get_current_pose();
    if (!current_pose_opt.has_value()) {
        return false;
    }

    auto current_pose = current_pose_opt.value();

    // 计算位置差距
    double dx = target_pose.pose.position.x - current_pose.pose.position.x;
    double dy = target_pose.pose.position.y - current_pose.pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    // 计算角度差距
    double target_yaw = PathConverter::quaternion_to_yaw_degrees(target_pose.pose.orientation);
    double current_yaw = PathConverter::quaternion_to_yaw_degrees(current_pose.pose.orientation);
    double angle_diff = std::abs(target_yaw - current_yaw);

    // 处理角度环形差异
    if (angle_diff > 180.0) {
        angle_diff = 360.0 - angle_diff;
    }

    return (distance <= goal_tolerance_xy_) && (angle_diff <= goal_tolerance_theta_ * 180.0 / M_PI);
}

std::optional<geometry_msgs::msg::PoseStamped> ROS2ZhongliBridge::get_current_pose() {
    try {
        auto transform = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = map_frame_;
        pose.header.stamp = this->get_clock()->now();
        pose.pose.position.x = transform.transform.translation.x;
        pose.pose.position.y = transform.transform.translation.y;
        pose.pose.position.z = transform.transform.translation.z;
        pose.pose.orientation = transform.transform.rotation;

        return pose;
    } catch (const tf2::TransformException& e) {
        log_error("TF2查找失败: " + std::string(e.what()));
        return std::nullopt;
    }
}

void ROS2ZhongliBridge::log_debug(const std::string& message) {
    RCLCPP_DEBUG(this->get_logger(), "%s", message.c_str());
}

void ROS2ZhongliBridge::log_error(const std::string& message) {
    RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
}

} // namespace zhongli_bridge