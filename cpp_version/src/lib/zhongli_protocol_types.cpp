#include "zhongli_protocol_types.hpp"
#include <chrono>
#include <random>

namespace zhongli_protocol {

std::string create_timestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()
    ) % 1000;

    std::stringstream ss;
    ss << std::put_time(std::gmtime(&time_t), "%Y-%m-%dT%H:%M:%S");
    ss << '.' << std::setfill('0') << std::setw(3) << ms.count() << 'Z';
    return ss.str();
}

std::string generate_trajectory_id(const std::string& robot_id) {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);

    // 生成时间戳部分 (YYYYMMDD)
    std::stringstream ss;
    ss << std::put_time(std::gmtime(&time_t), "%Y%m%d");
    std::string date_str = ss.str();

    // 生成唯一序列号
    auto ms_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()
    ).count();
    int sequence = static_cast<int>(ms_since_epoch % 1000000);

    ss.str("");
    ss << "traj-" << robot_id << "-" << date_str << "-"
       << std::setfill('0') << std::setw(6) << sequence;

    return ss.str();
}

std::string generate_action_id(const std::string& robot_id) {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);

    // 生成时间戳部分 (YYYYMMDD)
    std::stringstream ss;
    ss << std::put_time(std::gmtime(&time_t), "%Y%m%d");
    std::string date_str = ss.str();

    // 生成唯一序列号
    auto ms_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()
    ).count();
    int sequence = static_cast<int>(ms_since_epoch % 1000000);

    ss.str("");
    ss << "action-" << robot_id << "-" << date_str << "-"
       << std::setfill('0') << std::setw(6) << sequence;

    return ss.str();
}

std::string generate_task_id(const std::string& robot_id) {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);

    // 生成时间戳部分 (YYYYMMDD)
    std::stringstream ss;
    ss << std::put_time(std::gmtime(&time_t), "%Y%m%d");
    std::string date_str = ss.str();

    // 生成唯一序列号
    auto ms_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()
    ).count();
    int sequence = static_cast<int>(ms_since_epoch % 1000000);

    ss.str("");
    ss << "task-" << robot_id << "-" << date_str << "-"
       << std::setfill('0') << std::setw(6) << sequence;

    return ss.str();
}

} // namespace zhongli_protocol