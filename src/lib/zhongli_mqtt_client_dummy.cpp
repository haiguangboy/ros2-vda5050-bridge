#include "zhongli_mqtt_client.hpp"
#include <iostream>

namespace zhongli_protocol {

/**
 * @brief 虚拟MQTT客户端实现（用于没有MQTT库时的编译）
 */

ZhongliMqttClient::ZhongliMqttClient(const std::string& robot_id,
                                     const std::string& broker_host,
                                     int broker_port) {
    std::cout << "⚠️  使用虚拟MQTT客户端（paho-mqtt库未安装）" << std::endl;
}

ZhongliMqttClient::~ZhongliMqttClient() {
}

bool ZhongliMqttClient::connect() {
    std::cout << "⚠️  虚拟MQTT客户端：connect() - 无实际连接" << std::endl;
    return false;
}

void ZhongliMqttClient::disconnect() {
    std::cout << "⚠️  虚拟MQTT客户端：disconnect() - 无实际操作" << std::endl;
}

bool ZhongliMqttClient::is_connected() const {
    return false;
}

void ZhongliMqttClient::set_task_callback(TaskMessageCallback callback) {
    std::cout << "⚠️  虚拟MQTT客户端：set_task_callback() - 无实际操作" << std::endl;
}

void ZhongliMqttClient::set_trajectory_status_callback(TrajectoryStatusCallback callback) {
    std::cout << "⚠️  虚拟MQTT客户端：set_trajectory_status_callback() - 无实际操作" << std::endl;
}

void ZhongliMqttClient::set_action_status_callback(ActionStatusCallback callback) {
    std::cout << "⚠️  虚拟MQTT客户端：set_action_status_callback() - 无实际操作" << std::endl;
}

bool ZhongliMqttClient::publish_task_status(const TaskStatusMessage& message) {
    std::cout << "⚠️  虚拟MQTT客户端：publish_task_status() - 无实际发布" << std::endl;
    return false;
}

bool ZhongliMqttClient::publish_trajectory(const TrajectoryMessage& message) {
    std::cout << "📤 虚拟MQTT客户端：模拟发布轨迹 " << message.trajectoryId << std::endl;
    std::cout << "   轨迹点数: " << message.trajectoryPoints.size() << std::endl;
    return true; // 返回true让测试继续
}

bool ZhongliMqttClient::publish_action(const ActionMessage& message) {
    std::cout << "⚠️  虚拟MQTT客户端：publish_action() - 无实际发布" << std::endl;
    return false;
}

bool ZhongliMqttClient::publish_device_state(const DeviceStateMessage& message) {
    std::cout << "⚠️  虚拟MQTT客户端：publish_device_state() - 无实际发布" << std::endl;
    return false;
}

std::string ZhongliMqttClient::get_connection_status() const {
    return "虚拟客户端 - 未连接";
}

} // namespace zhongli_protocol