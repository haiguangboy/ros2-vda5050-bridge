#include <gtest/gtest.h>
#include "zhongli_protocol_types.hpp"
#include <nlohmann/json.hpp>

using namespace zhongli_protocol;

class ZhongliProtocolTypesTest : public ::testing::Test {
protected:
    void SetUp() override {
        robot_id_ = "test-robot-001";
    }

    std::string robot_id_;
};

// 测试基础数据类型
TEST_F(ZhongliProtocolTypesTest, TestPose) {
    Pose pose{10.2, 5.8, 90.0};

    // 测试JSON序列化
    nlohmann::json j = pose.to_json();
    EXPECT_DOUBLE_EQ(j["x"], 10.2);
    EXPECT_DOUBLE_EQ(j["y"], 5.8);
    EXPECT_DOUBLE_EQ(j["theta"], 90.0);

    // 测试JSON反序列化
    Pose pose2 = Pose::from_json(j);
    EXPECT_DOUBLE_EQ(pose2.x, 10.2);
    EXPECT_DOUBLE_EQ(pose2.y, 5.8);
    EXPECT_DOUBLE_EQ(pose2.theta, 90.0);
}

TEST_F(ZhongliProtocolTypesTest, TestContainerPose) {
    ContainerPose container_pose{10.5, 5.3, 0.1, 180.0};

    // 测试JSON序列化
    nlohmann::json j = container_pose.to_json();
    EXPECT_DOUBLE_EQ(j["x"], 10.5);
    EXPECT_DOUBLE_EQ(j["y"], 5.3);
    EXPECT_DOUBLE_EQ(j["z"], 0.1);
    EXPECT_DOUBLE_EQ(j["theta"], 180.0);

    // 测试JSON反序列化
    ContainerPose container_pose2 = ContainerPose::from_json(j);
    EXPECT_DOUBLE_EQ(container_pose2.x, 10.5);
    EXPECT_DOUBLE_EQ(container_pose2.y, 5.3);
    EXPECT_DOUBLE_EQ(container_pose2.z, 0.1);
    EXPECT_DOUBLE_EQ(container_pose2.theta, 180.0);
}

TEST_F(ZhongliProtocolTypesTest, TestForkliftState) {
    ForkliftState forklift{0.5, 150.2, 0.3, 0.8, true, "ready"};

    // 测试JSON序列化
    nlohmann::json j = forklift.to_json();
    EXPECT_DOUBLE_EQ(j["height"], 0.5);
    EXPECT_DOUBLE_EQ(j["weight"], 150.2);
    EXPECT_DOUBLE_EQ(j["lateralShift"], 0.3);
    EXPECT_DOUBLE_EQ(j["forwardExtension"], 0.8);
    EXPECT_TRUE(j["tiltBack"]);
    EXPECT_EQ(j["status"], "ready");

    // 测试JSON反序列化
    ForkliftState forklift2 = ForkliftState::from_json(j);
    EXPECT_DOUBLE_EQ(forklift2.height, 0.5);
    EXPECT_DOUBLE_EQ(forklift2.weight, 150.2);
    EXPECT_DOUBLE_EQ(forklift2.lateralShift, 0.3);
    EXPECT_DOUBLE_EQ(forklift2.forwardExtension, 0.8);
    EXPECT_TRUE(forklift2.tiltBack);
    EXPECT_EQ(forklift2.status, "ready");
}

TEST_F(ZhongliProtocolTypesTest, TestBatteryState) {
    BatteryState battery{85, false};

    // 测试JSON序列化
    nlohmann::json j = battery.to_json();
    EXPECT_EQ(j["level"], 85);
    EXPECT_FALSE(j["charging"]);

    // 测试JSON反序列化
    BatteryState battery2 = BatteryState::from_json(j);
    EXPECT_EQ(battery2.level, 85);
    EXPECT_FALSE(battery2.charging);
}

// 测试轨迹消息
TEST_F(ZhongliProtocolTypesTest, TestTrajectoryMessage) {
    std::vector<TrajectoryPoint> points = {
        {0.0, 0.0, 0.0},
        {5.0, 0.0, 0.0},
        {5.0, 3.0, 90.0},
        {8.0, 3.0, 0.0}
    };

    TrajectoryMessage traj_msg{
        create_timestamp(),
        generate_trajectory_id(robot_id_),
        points,
        1.5
    };

    // 测试JSON序列化
    std::string json_str = traj_msg.to_json_string();
    EXPECT_FALSE(json_str.empty());

    // 验证JSON内容
    nlohmann::json j = nlohmann::json::parse(json_str);
    EXPECT_EQ(j["trajectoryPoints"].size(), 4);
    EXPECT_DOUBLE_EQ(j["maxSpeed"], 1.5);
    EXPECT_FALSE(j["timestamp"].get<std::string>().empty());
    EXPECT_FALSE(j["trajectoryId"].get<std::string>().empty());

    // 测试JSON反序列化
    TrajectoryMessage traj_msg2 = TrajectoryMessage::from_json(j);
    EXPECT_EQ(traj_msg2.trajectoryPoints.size(), 4);
    EXPECT_DOUBLE_EQ(traj_msg2.maxSpeed, 1.5);
    EXPECT_EQ(traj_msg2.timestamp, traj_msg.timestamp);
    EXPECT_EQ(traj_msg2.trajectoryId, traj_msg.trajectoryId);
}

// 测试动作消息
TEST_F(ZhongliProtocolTypesTest, TestActionMessage) {
    ContainerPose container_pose{10.5, 5.3, 0.1, 180.0};

    ActionMessage action_msg{
        create_timestamp(),
        generate_action_id(robot_id_),
        "ground_pick",
        container_pose,
        "AGV-T300"
    };

    // 测试JSON序列化
    std::string json_str = action_msg.to_json_string();
    EXPECT_FALSE(json_str.empty());

    // 验证JSON内容
    nlohmann::json j = nlohmann::json::parse(json_str);
    EXPECT_EQ(j["actionType"], "ground_pick");
    EXPECT_EQ(j["containerType"], "AGV-T300");
    EXPECT_TRUE(j.contains("containerPose"));
    EXPECT_DOUBLE_EQ(j["containerPose"]["x"], 10.5);

    // 测试JSON反序列化
    ActionMessage action_msg2 = ActionMessage::from_json(j);
    EXPECT_EQ(action_msg2.actionType, "ground_pick");
    EXPECT_TRUE(action_msg2.containerType.has_value());
    EXPECT_EQ(action_msg2.containerType.value(), "AGV-T300");
    EXPECT_TRUE(action_msg2.containerPose.has_value());
    EXPECT_DOUBLE_EQ(action_msg2.containerPose.value().x, 10.5);
}

// 测试任务消息
TEST_F(ZhongliProtocolTypesTest, TestTaskMessage) {
    TaskMessage task_msg{
        create_timestamp(),
        generate_task_id(robot_id_),
        "A3 仓库区",
        "ground_pick",
        "B2 车间区",
        "unload"
    };

    // 测试JSON序列化
    std::string json_str = task_msg.to_json_string();
    EXPECT_FALSE(json_str.empty());

    // 验证JSON内容
    nlohmann::json j = nlohmann::json::parse(json_str);
    EXPECT_EQ(j["startArea"], "A3 仓库区");
    EXPECT_EQ(j["startAction"], "ground_pick");
    EXPECT_EQ(j["targetArea"], "B2 车间区");
    EXPECT_EQ(j["targetAction"], "unload");

    // 测试JSON反序列化
    TaskMessage task_msg2 = TaskMessage::from_json(j);
    EXPECT_EQ(task_msg2.startArea, "A3 仓库区");
    EXPECT_EQ(task_msg2.startAction, "ground_pick");
    EXPECT_EQ(task_msg2.targetArea, "B2 车间区");
    EXPECT_EQ(task_msg2.targetAction, "unload");
}

// 测试设备状态消息
TEST_F(ZhongliProtocolTypesTest, TestDeviceStateMessage) {
    Pose pose{10.2, 5.8, 90.0};
    ForkliftState forklift{0.5, 150.2, 0.3, 0.8, true, "ready"};
    BatteryState battery{85, false};

    std::vector<ErrorInfo> errors = {
        {1001, "传感器故障", create_timestamp()}
    };

    DeviceStateMessage state_msg{
        create_timestamp(),
        pose,
        forklift,
        battery,
        errors,
        "running"
    };

    // 测试JSON序列化
    std::string json_str = state_msg.to_json_string();
    EXPECT_FALSE(json_str.empty());

    // 验证JSON内容
    nlohmann::json j = nlohmann::json::parse(json_str);
    EXPECT_EQ(j["systemState"], "running");
    EXPECT_TRUE(j.contains("pose"));
    EXPECT_TRUE(j.contains("forkliftState"));
    EXPECT_TRUE(j.contains("battery"));
    EXPECT_EQ(j["errors"].size(), 1);
    EXPECT_EQ(j["errors"][0]["code"], 1001);
    EXPECT_EQ(j["errors"][0]["desc"], "传感器故障");
}

// 测试ID生成函数
TEST_F(ZhongliProtocolTypesTest, TestIdGeneration) {
    // 测试轨迹ID生成
    std::string traj_id1 = generate_trajectory_id(robot_id_);
    std::string traj_id2 = generate_trajectory_id(robot_id_);
    EXPECT_NE(traj_id1, traj_id2);  // ID应该不同
    EXPECT_TRUE(traj_id1.find("traj-") == 0);  // 应该以"traj-"开头
    EXPECT_TRUE(traj_id1.find(robot_id_) != std::string::npos);  // 应该包含机器人ID

    // 测试动作ID生成
    std::string action_id1 = generate_action_id(robot_id_);
    std::string action_id2 = generate_action_id(robot_id_);
    EXPECT_NE(action_id1, action_id2);  // ID应该不同
    EXPECT_TRUE(action_id1.find("action-") == 0);  // 应该以"action-"开头
    EXPECT_TRUE(action_id1.find(robot_id_) != std::string::npos);  // 应该包含机器人ID

    // 测试任务ID生成
    std::string task_id1 = generate_task_id(robot_id_);
    std::string task_id2 = generate_task_id(robot_id_);
    EXPECT_NE(task_id1, task_id2);  // ID应该不同
    EXPECT_TRUE(task_id1.find("task-") == 0);  // 应该以"task-"开头
    EXPECT_TRUE(task_id1.find(robot_id_) != std::string::npos);  // 应该包含机器人ID
}

// 测试时间戳生成
TEST_F(ZhongliProtocolTypesTest, TestTimestampGeneration) {
    std::string timestamp1 = create_timestamp();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    std::string timestamp2 = create_timestamp();

    EXPECT_NE(timestamp1, timestamp2);  // 时间戳应该不同
    EXPECT_TRUE(timestamp1.find('T') != std::string::npos);  // 应该包含ISO8601格式的'T'
    EXPECT_TRUE(timestamp1.find('Z') != std::string::npos);  // 应该以'Z'结尾
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}