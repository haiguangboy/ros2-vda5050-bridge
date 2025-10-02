#pragma once

#include <string>
#include <vector>
#include <optional>
#include <memory>
#include <nlohmann/json.hpp>
#include <chrono>
#include <iomanip>
#include <sstream>

namespace zhongli_protocol {

/**
 * @brief 机器人位姿信息
 */
struct Pose {
    double x;
    double y;
    double theta;  ///< 单位：弧度（范围：-π到+π）

    nlohmann::json to_json() const {
        return nlohmann::json{
            {"x", x},
            {"y", y},
            {"theta", theta}
        };
    }

    static Pose from_json(const nlohmann::json& j) {
        return Pose{
            j.at("x").get<double>(),
            j.at("y").get<double>(),
            j.at("theta").get<double>()
        };
    }
};

/**
 * @brief 容器/货物姿态
 */
struct ContainerPose {
    double x;
    double y;
    double z;
    double theta;  ///< 单位：弧度（范围：-π到+π）
    double width;  ///< 容器宽度，单位：米

    nlohmann::json to_json() const {
        return nlohmann::json{
            {"x", x},
            {"y", y},
            {"z", z},
            {"theta", theta},
            {"width", width}
        };
    }

    static ContainerPose from_json(const nlohmann::json& j) {
        return ContainerPose{
            j.at("x").get<double>(),
            j.at("y").get<double>(),
            j.at("z").get<double>(),
            j.at("theta").get<double>(),
            j.at("width").get<double>()
        };
    }
};

/**
 * @brief 货叉状态信息
 */
struct ForkliftState {
    double height;           ///< 货叉高度，单位：米
    double weight;           ///< 货叉负载重量，单位：千克
    double lateralShift;     ///< 货叉侧移量，单位：米
    double forwardExtension; ///< 货叉前伸量，单位：米
    bool tiltBack;          ///< 货叉是否后倾
    std::string status;     ///< 货叉工作状态: ready/moving/error

    nlohmann::json to_json() const {
        return nlohmann::json{
            {"height", height},
            {"weight", weight},
            {"lateralShift", lateralShift},
            {"forwardExtension", forwardExtension},
            {"tiltBack", tiltBack},
            {"status", status}
        };
    }

    static ForkliftState from_json(const nlohmann::json& j) {
        return ForkliftState{
            j.at("height").get<double>(),
            j.at("weight").get<double>(),
            j.at("lateralShift").get<double>(),
            j.at("forwardExtension").get<double>(),
            j.at("tiltBack").get<bool>(),
            j.at("status").get<std::string>()
        };
    }
};

/**
 * @brief 电池状态信息
 */
struct BatteryState {
    int level;      ///< 电池剩余电量，百分比 0-100
    bool charging;  ///< 是否正在充电

    nlohmann::json to_json() const {
        return nlohmann::json{
            {"level", level},
            {"charging", charging}
        };
    }

    static BatteryState from_json(const nlohmann::json& j) {
        return BatteryState{
            j.at("level").get<int>(),
            j.at("charging").get<bool>()
        };
    }
};

/**
 * @brief 错误信息
 */
struct ErrorInfo {
    int code;
    std::string desc;
    std::string timestamp;

    nlohmann::json to_json() const {
        return nlohmann::json{
            {"code", code},
            {"desc", desc},
            {"timestamp", timestamp}
        };
    }

    static ErrorInfo from_json(const nlohmann::json& j) {
        return ErrorInfo{
            j.at("code").get<int>(),
            j.at("desc").get<std::string>(),
            j.at("timestamp").get<std::string>()
        };
    }
};

/**
 * @brief 轨迹点上的动作信息
 */
struct TrajectoryAction {
    std::string actionType;                                ///< 动作类型
    std::optional<std::string> containerType;             ///< 容器类型（可选）
    std::optional<ContainerPose> containerPose;           ///< 容器位姿（可选）

    nlohmann::json to_json() const {
        nlohmann::json j = {
            {"actionType", actionType}
        };

        if (containerType.has_value()) {
            j["containerType"] = containerType.value();
        }
        if (containerPose.has_value()) {
            j["containerPose"] = containerPose.value().to_json();
        }

        return j;
    }

    static TrajectoryAction from_json(const nlohmann::json& j) {
        TrajectoryAction action;
        action.actionType = j.at("actionType").get<std::string>();

        if (j.contains("containerType")) {
            action.containerType = j.at("containerType").get<std::string>();
        }
        if (j.contains("containerPose")) {
            action.containerPose = ContainerPose::from_json(j.at("containerPose"));
        }

        return action;
    }
};

/**
 * @brief 轨迹点
 */
struct TrajectoryPoint {
    double x;                                              ///< X坐标，单位：米
    double y;                                              ///< Y坐标，单位：米
    double theta;                                          ///< 轨迹点航向角，单位：弧度（范围：-π到+π）
    double orientation;                                    ///< 运动方向，0:前向运动 -3.14:倒车 3.14:倒车
    double flag;                                           ///< 进入分支标志位，0:非进入分支 1:进行分支
    std::optional<TrajectoryAction> action;                ///< 该轨迹点上的动作（可选）

    nlohmann::json to_json() const {
        nlohmann::json j = {
            {"x", x},
            {"y", y},
            {"theta", theta},
            {"orientation", orientation},
            {"flag", flag}
        };

        if (action.has_value()) {
            j["action"] = action.value().to_json();
        } else {
            j["action"] = nullptr;
        }

        return j;
    }

    static TrajectoryPoint from_json(const nlohmann::json& j) {
        TrajectoryPoint point;
        point.x = j.at("x").get<double>();
        point.y = j.at("y").get<double>();
        point.theta = j.at("theta").get<double>();
        point.orientation = j.value("orientation", 0.0);
        point.flag = j.value("flag", 0.0);

        if (j.contains("action") && !j.at("action").is_null()) {
            point.action = TrajectoryAction::from_json(j.at("action"));
        }

        return point;
    }
};

/**
 * @brief 具身大脑下发轨迹指令消息
 */
struct TrajectoryMessage {
    std::string timestamp;
    std::string trajectoryId;
    std::vector<TrajectoryPoint> trajectoryPoints;
    double maxSpeed;  ///< 单位：米/秒

    std::string to_json_string() const {
        nlohmann::json points_json = nlohmann::json::array();
        for (const auto& point : trajectoryPoints) {
            points_json.push_back(point.to_json());
        }

        nlohmann::json j = {
            {"timestamp", timestamp},
            {"trajectoryId", trajectoryId},
            {"trajectoryPoints", points_json},
            {"maxSpeed", maxSpeed}
        };

        return j.dump(2);
    }

    static TrajectoryMessage from_json(const nlohmann::json& j) {
        std::vector<TrajectoryPoint> points;
        for (const auto& point_json : j.at("trajectoryPoints")) {
            points.push_back(TrajectoryPoint::from_json(point_json));
        }

        return TrajectoryMessage{
            j.at("timestamp").get<std::string>(),
            j.at("trajectoryId").get<std::string>(),
            points,
            j.at("maxSpeed").get<double>()
        };
    }
};

/**
 * @brief 车载小脑反馈轨迹状态消息
 */
struct TrajectoryStatusMessage {
    std::string timestamp;
    std::string trajectoryId;
    std::string status;  ///< pending/running/completed/failed
    std::optional<int> currentPointIndex;
    int errorCode = 0;
    std::string errorDesc = "";
    std::optional<std::string> estimatedFinishTime;
    std::optional<std::string> finishTime;

    std::string to_json_string() const {
        nlohmann::json j = {
            {"timestamp", timestamp},
            {"trajectoryId", trajectoryId},
            {"status", status},
            {"errorCode", errorCode},
            {"errorDesc", errorDesc}
        };

        if (currentPointIndex.has_value()) {
            j["currentPointIndex"] = currentPointIndex.value();
        }
        if (estimatedFinishTime.has_value()) {
            j["estimatedFinishTime"] = estimatedFinishTime.value();
        }
        if (finishTime.has_value()) {
            j["finishTime"] = finishTime.value();
        }

        return j.dump(2);
    }

    static TrajectoryStatusMessage from_json(const nlohmann::json& j) {
        TrajectoryStatusMessage msg;
        msg.timestamp = j.at("timestamp").get<std::string>();
        msg.trajectoryId = j.at("trajectoryId").get<std::string>();

        // 支持两种状态字段格式：新格式用"status"，旧格式用"trajectoryStatus"
        if (j.contains("status")) {
            msg.status = j.at("status").get<std::string>();
        } else if (j.contains("trajectoryStatus")) {
            std::string trajectory_status = j.at("trajectoryStatus").get<std::string>();
            // 映射旧格式状态到新格式
            if (trajectory_status == "FINISHED" || trajectory_status == "COMPLETED") {
                msg.status = "completed";
            } else if (trajectory_status == "RUNNING" || trajectory_status == "EXECUTING") {
                msg.status = "running";
            } else if (trajectory_status == "FAILED" || trajectory_status == "ERROR") {
                msg.status = "failed";
            } else if (trajectory_status == "PENDING" || trajectory_status == "ACCEPTED") {
                msg.status = "pending";
            } else {
                msg.status = trajectory_status; // 保持原样
            }
        } else {
            msg.status = "unknown";
        }

        msg.errorCode = j.value("errorCode", 0);
        msg.errorDesc = j.value("errorDesc", std::string(""));

        // 支持resultDescription作为错误描述的备选
        if (msg.errorDesc.empty() && j.contains("resultDescription")) {
            msg.errorDesc = j.at("resultDescription").get<std::string>();
        }

        // 支持两种当前位置格式
        if (j.contains("currentPointIndex")) {
            msg.currentPointIndex = j.at("currentPointIndex").get<int>();
        } else if (j.contains("currentPosition")) {
            // 如果有currentPosition对象，我们可以提取一些信息但不直接转换为索引
            // 这里暂时不设置currentPointIndex，因为位置坐标不等同于点索引
        }

        if (j.contains("estimatedFinishTime")) {
            msg.estimatedFinishTime = j.at("estimatedFinishTime").get<std::string>();
        }
        if (j.contains("finishTime")) {
            msg.finishTime = j.at("finishTime").get<std::string>();
        }

        return msg;
    }
};

/**
 * @brief 具身大脑下发动作指令消息
 */
struct ActionMessage {
    std::string timestamp;
    std::string actionId;
    std::string actionType;  ///< ground_pick/ground_place/load/unload/pub_params
    std::optional<ContainerPose> containerPose;
    std::optional<std::string> containerType;

    std::string to_json_string() const {
        nlohmann::json j = {
            {"timestamp", timestamp},
            {"actionId", actionId},
            {"actionType", actionType}
        };

        if (containerPose.has_value()) {
            j["containerPose"] = containerPose.value().to_json();
        }
        if (containerType.has_value()) {
            j["containerType"] = containerType.value();
        }

        return j.dump(2);
    }

    static ActionMessage from_json(const nlohmann::json& j) {
        std::optional<ContainerPose> container_pose;
        if (j.contains("containerPose")) {
            container_pose = ContainerPose::from_json(j.at("containerPose"));
        }

        std::optional<std::string> container_type;
        if (j.contains("containerType")) {
            container_type = j.at("containerType").get<std::string>();
        }

        return ActionMessage{
            j.at("timestamp").get<std::string>(),
            j.at("actionId").get<std::string>(),
            j.at("actionType").get<std::string>(),
            container_pose,
            container_type
        };
    }
};

/**
 * @brief 车载小脑反馈动作状态消息
 */
struct ActionStatusMessage {
    std::string timestamp;
    std::string actionId;
    std::string status;  ///< success/failed
    int errorCode = 0;
    std::string errorDesc = "";
    std::optional<std::string> finishTime;

    std::string to_json_string() const {
        nlohmann::json j = {
            {"timestamp", timestamp},
            {"actionId", actionId},
            {"status", status},
            {"errorCode", errorCode},
            {"errorDesc", errorDesc}
        };

        if (finishTime.has_value()) {
            j["finishTime"] = finishTime.value();
        }

        return j.dump(2);
    }
};

/**
 * @brief 调度下发任务消息
 */
struct TaskMessage {
    std::string timestamp;
    std::string taskId;
    std::string startArea;
    std::string startAction;   ///< ground_pick/ground_place/load/unload/pub_params
    std::string targetArea;
    std::string targetAction;  ///< ground_pick/ground_place/load/unload/pub_params

    std::string to_json_string() const {
        nlohmann::json j = {
            {"timestamp", timestamp},
            {"taskId", taskId},
            {"startArea", startArea},
            {"startAction", startAction},
            {"targetArea", targetArea},
            {"targetAction", targetAction}
        };

        return j.dump(2);
    }

    static TaskMessage from_json(const nlohmann::json& j) {
        return TaskMessage{
            j.at("timestamp").get<std::string>(),
            j.at("taskId").get<std::string>(),
            j.at("startArea").get<std::string>(),
            j.at("startAction").get<std::string>(),
            j.at("targetArea").get<std::string>(),
            j.at("targetAction").get<std::string>()
        };
    }
};

/**
 * @brief 机器人上报任务状态消息
 */
struct TaskStatusMessage {
    std::string timestamp;
    std::string taskId;
    std::string status;  ///< success/failed
    std::string finishTime;
    std::string reason = "";  ///< 失败原因

    std::string to_json_string() const {
        nlohmann::json j = {
            {"timestamp", timestamp},
            {"taskId", taskId},
            {"status", status},
            {"finishTime", finishTime},
            {"reason", reason}
        };

        return j.dump(2);
    }
};

/**
 * @brief 车载网关上报设备状态消息
 */
struct DeviceStateMessage {
    std::string timestamp;
    Pose pose;
    ForkliftState forkliftState;
    BatteryState battery;
    std::vector<ErrorInfo> errors;
    std::string systemState;  ///< idle/running/paused/fault

    std::string to_json_string() const {
        nlohmann::json errors_json = nlohmann::json::array();
        for (const auto& error : errors) {
            errors_json.push_back(error.to_json());
        }

        nlohmann::json j = {
            {"timestamp", timestamp},
            {"pose", pose.to_json()},
            {"forkliftState", forkliftState.to_json()},
            {"battery", battery.to_json()},
            {"errors", errors_json},
            {"systemState", systemState}
        };

        return j.dump(2);
    }
};

/**
 * @brief 创建ISO8601 UTC时间戳
 */
std::string create_timestamp();

/**
 * @brief 生成轨迹ID
 */
std::string generate_trajectory_id(const std::string& robot_id);

/**
 * @brief 生成动作ID
 */
std::string generate_action_id(const std::string& robot_id);

/**
 * @brief 生成任务ID
 */
std::string generate_task_id(const std::string& robot_id);

} // namespace zhongli_protocol