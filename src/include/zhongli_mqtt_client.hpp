#pragma once

#include <string>
#include <functional>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <unordered_map>
#ifdef MQTT_ENABLED
#include <mosquitto.h>
#endif
#include "zhongli_protocol_types.hpp"

namespace zhongli_protocol {

/**
 * @brief 消息回调函数类型定义
 */
using TaskMessageCallback = std::function<void(const TaskMessage&)>;
using TrajectoryStatusCallback = std::function<void(const TrajectoryStatusMessage&)>;
using ActionStatusCallback = std::function<void(const ActionStatusMessage&)>;

/**
 * @brief 中力具身机器人MQTT客户端
 *
 * 实现与中力具身机器人系统的MQTT通信，包括：
 * - EP/{direction}/{robotId}/{module}/{topic} 主题结构
 * - 双向消息传递（控制器 <-> ROS2）
 * - 实时状态反馈
 */
class ZhongliMqttClient {
public:
    /**
     * @brief 构造函数
     *
     * @param robot_id 机器人ID
     * @param broker_host MQTT代理主机地址
     * @param broker_port MQTT代理端口
     */
    ZhongliMqttClient(const std::string& robot_id,
                      const std::string& broker_host = "localhost",
                      int broker_port = 1883);

    /**
     * @brief 析构函数
     */
    ~ZhongliMqttClient();

    /**
     * @brief 连接到MQTT代理
     *
     * @return true 连接成功
     * @return false 连接失败
     */
    bool connect();

    /**
     * @brief 断开MQTT连接
     */
    void disconnect();

    /**
     * @brief 检查连接状态
     *
     * @return true 已连接
     * @return false 未连接
     */
    bool is_connected() const;

    /**
     * @brief 设置任务消息回调函数
     *
     * @param callback 回调函数
     */
    void set_task_callback(TaskMessageCallback callback);

    /**
     * @brief 设置轨迹状态回调函数
     *
     * @param callback 回调函数
     */
    void set_trajectory_status_callback(TrajectoryStatusCallback callback);

    /**
     * @brief 设置动作状态回调函数
     *
     * @param callback 回调函数
     */
    void set_action_status_callback(ActionStatusCallback callback);

    // 发布消息方法

    /**
     * @brief 发布任务状态消息
     *
     * @param message 任务状态消息
     * @return true 发布成功
     * @return false 发布失败
     */
    bool publish_task_status(const TaskStatusMessage& message);

    /**
     * @brief 发布轨迹指令消息
     *
     * @param message 轨迹指令消息
     * @return true 发布成功
     * @return false 发布失败
     */
    bool publish_trajectory(const TrajectoryMessage& message);

    /**
     * @brief 发布动作指令消息
     *
     * @param message 动作指令消息
     * @return true 发布成功
     * @return false 发布失败
     */
    bool publish_action(const ActionMessage& message);

    /**
     * @brief 发布设备状态消息
     *
     * @param message 设备状态消息
     * @return true 发布成功
     * @return false 发布失败
     */
    bool publish_device_state(const DeviceStateMessage& message);

    /**
     * @brief 获取机器人ID
     *
     * @return std::string 机器人ID
     */
    const std::string& get_robot_id() const { return robot_id_; }

    /**
     * @brief 获取连接状态字符串
     *
     * @return std::string 连接状态描述
     */
    std::string get_connection_status() const;

private:
#ifdef MQTT_ENABLED
    /**
     * @brief 消息到达回调（内部使用）
     */
    static void on_message_callback(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message);

    /**
     * @brief 连接回调（内部使用）
     */
    static void on_connect_callback(struct mosquitto *mosq, void *obj, int result);

    /**
     * @brief 断开连接回调（内部使用）
     */
    static void on_disconnect_callback(struct mosquitto *mosq, void *obj, int result);
#endif

    /**
     * @brief 发布消息的通用方法
     *
     * @param topic 主题
     * @param payload 消息内容
     * @param qos 服务质量等级
     * @return true 发布成功
     * @return false 发布失败
     */
    bool publish_message(const std::string& topic,
                        const std::string& payload,
                        int qos = 1);

    /**
     * @brief 订阅主题的通用方法
     *
     * @param topic 主题
     * @param qos 服务质量等级
     * @return true 订阅成功
     * @return false 订阅失败
     */
    bool subscribe_topic(const std::string& topic, int qos = 1);

    /**
     * @brief 生成主题名称
     *
     * @param direction 方向 (master/robotId)
     * @param module 模块 (embrain/cerebellum)
     * @param topic 主题名称
     * @return std::string 完整主题路径
     */
    std::string build_topic(const std::string& direction,
                           const std::string& module,
                           const std::string& topic) const;

    // 成员变量
    std::string robot_id_;
    std::string broker_host_;
    int broker_port_;
    std::string client_id_;

#ifdef MQTT_ENABLED
    struct mosquitto *mosq_;
#else
    void *mosq_; // dummy pointer for consistency
#endif

    // 回调函数
    TaskMessageCallback task_callback_;
    TrajectoryStatusCallback trajectory_status_callback_;
    ActionStatusCallback action_status_callback_;

    // 线程同步
    mutable std::mutex mutex_;
    std::condition_variable connection_cv_;
    bool connected_;

    // 主题名称缓存
    std::string task_subscribe_topic_;          // EP/master/{robotId}/task
    std::string task_status_publish_topic_;     // EP/master/{robotId}/task_status
    std::string trajectory_publish_topic_;      // EP/{robotId}/embrain/cerebellum/trajectory
    std::string trajectory_status_subscribe_topic_; // EP/{robotId}/cerebellum/embrain/trajectory_status
    std::string action_publish_topic_;          // EP/{robotId}/embrain/cerebellum/action
    std::string action_status_subscribe_topic_; // EP/{robotId}/cerebellum/embrain/action_status
    std::string state_publish_topic_;           // EP/master/{robotId}/state

    // 线程管理
    std::thread network_thread_;
};

} // namespace zhongli_protocol