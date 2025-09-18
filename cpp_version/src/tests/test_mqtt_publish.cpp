#include <iostream>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <string>
#include <sstream>
#include "../include/zhongli_protocol_types.hpp"

// Test real MQTT publishing using mosquitto_pub command
class MQTTPublisher {
private:
    std::string broker_host_;
    int broker_port_;
    std::string robot_id_;

public:
    MQTTPublisher(const std::string& host, int port, const std::string& robot_id)
        : broker_host_(host), broker_port_(port), robot_id_(robot_id) {}

    bool publish_trajectory(const zhongli_protocol::TrajectoryMessage& trajectory) {
        // 构建MQTT主题：EP/downstream/{robotId}/path_planning/trajectory_message
        std::string topic = "EP/downstream/" + robot_id_ + "/path_planning/trajectory_message";

        // 序列化消息为JSON
        std::string json_message = zhongli_protocol::trajectory_to_json(trajectory);

        // 使用mosquitto_pub发布消息
        std::stringstream cmd;
        cmd << "mosquitto_pub -h " << broker_host_
            << " -p " << broker_port_
            << " -t \"" << topic << "\""
            << " -m '" << json_message << "'";

        std::cout << "📤 发布MQTT消息到主题: " << topic << std::endl;
        std::cout << "🔧 执行命令: " << cmd.str() << std::endl;

        int result = system(cmd.str().c_str());

        if (result == 0) {
            std::cout << "✅ MQTT消息发布成功" << std::endl;
            return true;
        } else {
            std::cout << "❌ MQTT消息发布失败，返回代码: " << result << std::endl;
            return false;
        }
    }

    bool publish_device_state(const zhongli_protocol::DeviceStateMessage& state) {
        // 构建MQTT主题：EP/upstream/{robotId}/device_state/realtime_status
        std::string topic = "EP/upstream/" + robot_id_ + "/device_state/realtime_status";

        // 序列化消息为JSON
        std::string json_message = zhongli_protocol::device_state_to_json(state);

        // 使用mosquitto_pub发布消息
        std::stringstream cmd;
        cmd << "mosquitto_pub -h " << broker_host_
            << " -p " << broker_port_
            << " -t \"" << topic << "\""
            << " -m '" << json_message << "'";

        std::cout << "📤 发布设备状态到主题: " << topic << std::endl;

        int result = system(cmd.str().c_str());

        if (result == 0) {
            std::cout << "✅ 设备状态发布成功" << std::endl;
            return true;
        } else {
            std::cout << "❌ 设备状态发布失败，返回代码: " << result << std::endl;
            return false;
        }
    }
};

int main() {
    std::cout << "🧪 测试真实MQTT发布功能" << std::endl;
    std::cout << "==================================" << std::endl;

    // 创建MQTT发布器
    MQTTPublisher publisher("localhost", 1883, "robot-001");

    // 创建测试轨迹消息
    zhongli_protocol::TrajectoryMessage trajectory;
    trajectory.trajectoryId = "test-traj-" + zhongli_protocol::create_timestamp();
    trajectory.timestamp = zhongli_protocol::create_timestamp();
    trajectory.version = "1.0";

    // 添加几个轨迹点
    for (int i = 0; i < 3; ++i) {
        zhongli_protocol::TrajectoryPoint point;
        point.id = "point-" + std::to_string(i);
        point.x = i * 1.0;
        point.y = i * 0.5;
        point.theta = i * 30.0;  // 每点旋转30度
        point.speed = 1.0;
        point.actions.push_back("wait:1000");  // 等待1秒
        trajectory.trajectoryPoints.push_back(point);
    }

    std::cout << "\n📋 测试轨迹消息内容:" << std::endl;
    std::cout << "  轨迹ID: " << trajectory.trajectoryId << std::endl;
    std::cout << "  点数量: " << trajectory.trajectoryPoints.size() << std::endl;

    // 测试轨迹发布
    std::cout << "\n🎯 测试1: 发布轨迹消息" << std::endl;
    bool trajectory_success = publisher.publish_trajectory(trajectory);

    // 等待一下
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 创建测试设备状态消息
    zhongli_protocol::DeviceStateMessage state;
    state.timestamp = zhongli_protocol::create_timestamp();
    state.pose.x = 1.5;
    state.pose.y = 2.0;
    state.pose.theta = 45.0;
    state.forkliftState.height = 0.1;
    state.forkliftState.status = "ready";
    state.battery.level = 88;
    state.battery.charging = false;
    state.systemState = "running";

    std::cout << "\n📋 测试设备状态内容:" << std::endl;
    std::cout << "  位置: (" << state.pose.x << ", " << state.pose.y << ", " << state.pose.theta << "°)" << std::endl;
    std::cout << "  电池: " << state.battery.level << "%" << std::endl;
    std::cout << "  系统状态: " << state.systemState << std::endl;

    // 测试状态发布
    std::cout << "\n🎯 测试2: 发布设备状态" << std::endl;
    bool state_success = publisher.publish_device_state(state);

    // 总结测试结果
    std::cout << "\n📊 测试结果总结:" << std::endl;
    std::cout << "  轨迹发布: " << (trajectory_success ? "✅ 成功" : "❌ 失败") << std::endl;
    std::cout << "  状态发布: " << (state_success ? "✅ 成功" : "❌ 失败") << std::endl;

    if (trajectory_success && state_success) {
        std::cout << "\n🎉 所有MQTT发布测试通过！" << std::endl;
        std::cout << "   现在可以在EMQX控制台或mosquitto_sub中查看发布的消息" << std::endl;
        std::cout << "\n💡 监听命令示例:" << std::endl;
        std::cout << "   mosquitto_sub -h localhost -p 1883 -t 'EP/downstream/robot-001/+/+'" << std::endl;
        std::cout << "   mosquitto_sub -h localhost -p 1883 -t 'EP/upstream/robot-001/+/+'" << std::endl;
        return 0;
    } else {
        std::cout << "\n⚠️  部分测试失败，请检查EMQX服务器状态" << std::endl;
        return 1;
    }
}