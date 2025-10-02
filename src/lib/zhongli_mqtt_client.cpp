#include "zhongli_mqtt_client.hpp"
#include <nlohmann/json.hpp>
#include <iostream>
#include <chrono>
#include <sstream>
#include <cstring>

namespace zhongli_protocol {

ZhongliMqttClient::ZhongliMqttClient(const std::string& robot_id,
                                     const std::string& broker_host,
                                     int broker_port)
    : robot_id_(robot_id)
    , broker_host_(broker_host)
    , broker_port_(broker_port)
    , mosq_(nullptr)
    , connected_(false) {

    // 生成客户端ID
    std::stringstream ss;
    ss << "zhongli_bridge_" << robot_id_ << "_" << std::time(nullptr);
    client_id_ = ss.str();

    // 初始化mosquitto库
    mosquitto_lib_init();

    // 创建mosquitto客户端实例
    mosq_ = mosquitto_new(client_id_.c_str(), true, this);
    if (!mosq_) {
        std::cerr << "❌ 创建mosquitto客户端失败" << std::endl;
        return;
    }

    // 设置回调函数
    mosquitto_connect_callback_set(mosq_, on_connect_callback);
    mosquitto_disconnect_callback_set(mosq_, on_disconnect_callback);
    mosquitto_message_callback_set(mosq_, on_message_callback);

    // 构建主题名称
    task_subscribe_topic_ = build_topic("master", robot_id_, "task");
    task_status_publish_topic_ = build_topic("master", robot_id_, "task_status");
    trajectory_publish_topic_ = build_topic(robot_id_, "embrain/cerebellum", "trajectory");
    trajectory_status_subscribe_topic_ = build_topic(robot_id_, "cerebellum/embrain", "trajectory_status");
    action_publish_topic_ =action_publish_topic_ =  build_topic(robot_id_, "embrain/cerebellum", "action");
    action_status_subscribe_topic_ = build_topic(robot_id_, "cerebellum/embrain", "action_status");
    state_publish_topic_ = build_topic("master", robot_id_, "state");

    std::cout << "✅ 中力MQTT客户端已创建 - Robot ID: " << robot_id_ << std::endl;
}

ZhongliMqttClient::~ZhongliMqttClient() {
    disconnect();
    if (mosq_) {
        mosquitto_destroy(mosq_);
    }
    mosquitto_lib_cleanup();
}

bool ZhongliMqttClient::connect() {
    if (!mosq_) {
        std::cerr << "❌ mosquitto客户端未初始化" << std::endl;
        return false;
    }

    std::cout << "🔌 正在连接到EMQX服务器: " << broker_host_ << ":" << broker_port_ << std::endl;
    std::cout << "📋 MQTT配置详情:" << std::endl;
    std::cout << "   - Robot ID: " << robot_id_ << std::endl;
    std::cout << "   - Broker地址: " << broker_host_ << std::endl;
    std::cout << "   - Broker端口: " << broker_port_ << std::endl;
    std::cout << "   - 客户端ID: zhongli_" << robot_id_ << std::endl;

    int rc = mosquitto_connect(mosq_, broker_host_.c_str(), broker_port_, 60);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "❌ 连接失败: " << mosquitto_strerror(rc) << std::endl;
        return false;
    }

    // 启动网络循环线程
    rc = mosquitto_loop_start(mosq_);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "❌ 启动网络循环失败: " << mosquitto_strerror(rc) << std::endl;
        return false;
    }

    // 等待连接建立
    std::unique_lock<std::mutex> lock(mutex_);
    auto timeout = std::chrono::seconds(10);
    if (!connection_cv_.wait_for(lock, timeout, [this] { return connected_; })) {
        std::cerr << "❌ 连接超时" << std::endl;
        return false;
    }

    std::cout << "✅ 已连接到EMQX服务器" << std::endl;

    // 打印MQTT主题信息
    std::cout << "📡 MQTT主题配置:" << std::endl;
    std::cout << "   订阅主题:" << std::endl;
    std::cout << "     - 任务主题: " << task_subscribe_topic_ << std::endl;
    std::cout << "     - 轨迹状态: " << trajectory_status_subscribe_topic_ << std::endl;
    std::cout << "     - 动作状态: " << action_status_subscribe_topic_ << std::endl;
    std::cout << "   发布主题:" << std::endl;
    std::cout << "     - 轨迹指令: " << trajectory_publish_topic_ << std::endl;
    std::cout << "     - 动作指令: " << action_publish_topic_ << std::endl;
    std::cout << "     - 设备状态: " << state_publish_topic_ << std::endl;

    // 订阅相关主题
    subscribe_topic(task_subscribe_topic_);
    subscribe_topic(trajectory_status_subscribe_topic_);
    subscribe_topic(action_status_subscribe_topic_);

    return true;
}

void ZhongliMqttClient::disconnect() {
    if (mosq_ && connected_) {
        std::cout << "🔌 正在断开MQTT连接..." << std::endl;
        mosquitto_loop_stop(mosq_, false);
        mosquitto_disconnect(mosq_);

        std::lock_guard<std::mutex> lock(mutex_);
        connected_ = false;
    }
}

bool ZhongliMqttClient::is_connected() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return connected_;
}

void ZhongliMqttClient::set_task_callback(TaskMessageCallback callback) {
    task_callback_ = callback;
}

void ZhongliMqttClient::set_trajectory_status_callback(TrajectoryStatusCallback callback) {
    trajectory_status_callback_ = callback;
}

void ZhongliMqttClient::set_action_status_callback(ActionStatusCallback callback) {
    action_status_callback_ = callback;
}

bool ZhongliMqttClient::publish_task_status(const TaskStatusMessage& message) {
    return publish_message(task_status_publish_topic_, message.to_json_string());
}

bool ZhongliMqttClient::publish_trajectory(const TrajectoryMessage& message) {
    std::cout << "📤 发布轨迹指令到: " << trajectory_publish_topic_ << std::endl;
    std::cout << "   轨迹ID: " << message.trajectoryId << std::endl;
    std::cout << "   轨迹点数: " << message.trajectoryPoints.size() << std::endl;

    return publish_message(trajectory_publish_topic_, message.to_json_string());
}

bool ZhongliMqttClient::publish_action(const ActionMessage& message) {
    return publish_message(action_publish_topic_, message.to_json_string());
}

bool ZhongliMqttClient::publish_device_state(const DeviceStateMessage& message) {
    return publish_message(state_publish_topic_, message.to_json_string());
}

std::string ZhongliMqttClient::get_connection_status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (connected_) {
        return "已连接到EMQX " + broker_host_ + ":" + std::to_string(broker_port_);
    } else {
        return "未连接";
    }
}

// 静态回调函数实现
void ZhongliMqttClient::on_connect_callback(struct mosquitto *mosq, void *obj, int result) {
    ZhongliMqttClient* client = static_cast<ZhongliMqttClient*>(obj);
    if (!client) return;

    std::lock_guard<std::mutex> lock(client->mutex_);
    if (result == 0) {
        client->connected_ = true;
        std::cout << "✅ MQTT连接建立成功" << std::endl;
    } else {
        client->connected_ = false;
        std::cerr << "❌ MQTT连接失败: " << mosquitto_connack_string(result) << std::endl;
    }
    client->connection_cv_.notify_all();
}

void ZhongliMqttClient::on_disconnect_callback(struct mosquitto *mosq, void *obj, int result) {
    ZhongliMqttClient* client = static_cast<ZhongliMqttClient*>(obj);
    if (!client) return;

    std::lock_guard<std::mutex> lock(client->mutex_);
    client->connected_ = false;
    std::cout << "🔌 MQTT连接已断开" << std::endl;
}

void ZhongliMqttClient::on_message_callback(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message) {
    ZhongliMqttClient* client = static_cast<ZhongliMqttClient*>(obj);
    if (!client || !message || !message->payload) return;

    std::string topic(message->topic);
    std::string payload(static_cast<char*>(message->payload), message->payloadlen);

    std::cout << "📩 收到MQTT消息 - 主题: " << topic << std::endl;

    try {
        nlohmann::json json_msg = nlohmann::json::parse(payload);

        // 根据主题分发消息
        if (topic == client->task_subscribe_topic_ && client->task_callback_) {
            TaskMessage task_msg = TaskMessage::from_json(json_msg);
            client->task_callback_(task_msg);
        } else if (topic == client->trajectory_status_subscribe_topic_ && client->trajectory_status_callback_) {
            std::cout << "收到轨迹状态消息: " << json_msg.dump() << std::endl;
            try {
                TrajectoryStatusMessage trajectory_msg = TrajectoryStatusMessage::from_json(json_msg);
                std::cout << "✅ 轨迹状态消息解析成功: " << trajectory_msg.trajectoryId
                         << " - " << trajectory_msg.status << std::endl;
                client->trajectory_status_callback_(trajectory_msg);
            } catch (const std::exception& parse_error) {
                std::cerr << "❌ 轨迹状态消息解析失败: " << parse_error.what() << std::endl;
                std::cerr << "   原始JSON: " << json_msg.dump() << std::endl;
            }
        } else if (topic == client->action_status_subscribe_topic_ && client->action_status_callback_) {
            // 暂时跳过动作状态消息解析
            std::cout << "收到动作状态消息: " << json_msg.dump() << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "❌ 解析MQTT消息失败: " << e.what() << std::endl;
    }
}

bool ZhongliMqttClient::publish_message(const std::string& topic, const std::string& payload, int qos) {
    if (!mosq_ || !connected_) {
        std::cerr << "❌ MQTT客户端未连接，无法发布消息" << std::endl;
        return false;
    }

    int rc = mosquitto_publish(mosq_, nullptr, topic.c_str(), payload.length(), payload.c_str(), qos, false);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "❌ 发布消息失败: " << mosquitto_strerror(rc) << std::endl;
        return false;
    }

    return true;
}

bool ZhongliMqttClient::subscribe_topic(const std::string& topic, int qos) {
    if (!mosq_ || !connected_) {
        std::cerr << "❌ MQTT客户端未连接，无法订阅主题" << std::endl;
        return false;
    }

    int rc = mosquitto_subscribe(mosq_, nullptr, topic.c_str(), qos);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "❌ 订阅主题失败: " << mosquitto_strerror(rc) << std::endl;
        return false;
    }

    std::cout << "📝 已订阅主题: " << topic << std::endl;
    return true;
}

std::string ZhongliMqttClient::build_topic(const std::string& direction,
                                          const std::string& module,
                                          const std::string& topic) const {
    return "EP/" + direction + "/" + module + "/" + topic;
}

} // namespace zhongli_protocol
