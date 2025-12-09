// include/nlp_drone_control/common_types.h
#pragma once

#include <string>
#include <vector>
#include <deque>
#include <map>
#include <mutex>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

/**
 * @brief Shared enums and data structures between Planner and Executor nodes.
 */

enum DroneState {
    GROUNDED, TAKING_OFF, HOVERING, FLYING, LANDING, PERFORMING_ACTION, UNKNOWN
};

enum class ReplanMode { APPEND, REPLACE };

struct NamedPoint {
    std::string name;
    geometry_msgs::Point point;
};

struct SimplifiedBoundary {
    geometry_msgs::Point min;
    geometry_msgs::Point max;
};

struct POIInfoData {
    std::string name;
    std::string type;
    std::string description;
    geometry_msgs::Point position;                // 原始位置
    std::vector<NamedPoint> candidate_points;     // 可能的交互点

    // 新增：最佳接近点
    geometry_msgs::Point best_approach;
    bool has_best_approach = false;               // 标记是否有 best_approach

    // 原来的简化边界
    bool has_simplified_boundary = false;
    SimplifiedBoundary simplified_boundary;
};


/**
 * @brief Stores runtime state of each drone (pose, status, queue, macro task info).
 */
struct DroneContext {
    std::string id;
    std::string status_str = "UNKNOWN";
    DroneState state_enum = UNKNOWN;
    geometry_msgs::PoseStamped pose;

    geometry_msgs::Point launch_position;
    bool has_recorded_launch_position = false;
    ros::Time navigation_start_time_;

    std::deque<json> tool_call_queue;
    bool is_task_executing = false;
    std::mutex context_data_mutex;

    bool is_in_macro_task = false;
    std::string current_macro_name;
    int macro_task_step = 0;
    json macro_task_data;
    std::vector<sensor_msgs::Image> collected_images_for_vlm;

    json current_executing_tool_call; ///< Currently running tool call
    
    int navigation_retry_count = 0; // 导航重试计数器
    json current_nav_task;          // 保存当前正在尝试的导航任务
    std::string current_tool_name; // <--- 新增
};

/**
 * @brief Represents the next action to take when advancing a macro task.
 */
struct NextMacroAction {
    bool is_valid = false;
    bool is_capture_action = false;
    std::string action_type;
    std::string params_json;
    std::string capture_filename;
};

