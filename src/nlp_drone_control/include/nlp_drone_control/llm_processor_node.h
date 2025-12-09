#ifndef LLM_PROCESSOR_NODE_H
#define LLM_PROCESSOR_NODE_H

// --- Standard C++ Library Includes ---
#include <string>
#include <vector>
#include <sstream>
#include <map>
#include <deque>
#include <thread>
#include <mutex>
#include <filesystem> // For std::filesystem
#include <chrono>     // For std::chrono
#include <iomanip>    // For std::put_time
#include <cfloat>     // For DBL_MAX
#include <atomic>

// --- ROS Includes ---
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <XmlRpcValue.h>
#include <std_msgs/Empty.h>

// --- TF2 Includes for Geometry ---
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// --- External Libraries ---
#include <cpr/cpr.h>
#include <nlohmann/json.hpp>
#include <boost/bind.hpp>

// --- Project-Specific Service and Message Includes ---
#include "nlp_drone_control/ExecuteDroneAction.h"
#include "vlm_service/DescribeScene.h"
#include "vlm_service/CaptureImage.h"

// --- Convenience Alias ---
using json = nlohmann::json;

// --- Enumerations ---
enum DroneState {
    GROUNDED, TAKING_OFF, HOVERING, FLYING, LANDING, PERFORMING_ACTION, UNKNOWN
};

enum class ReplanMode {
    APPEND, REPLACE
};

// --- Data Structures ---
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
    geometry_msgs::Point position;
    std::vector<NamedPoint> candidate_points;
    bool has_simplified_boundary = false;
    SimplifiedBoundary simplified_boundary;
};

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
    
    json current_executing_tool_call; // 保存正在执行的整个tool_call JSON
};
// 用于在宏任务中传递下一步动作的结构体
struct NextMacroAction {
    bool is_valid = false;          // 是否有有效动作
    bool is_capture_action = false; // 特殊标记，表示这是个拍照动作
    std::string action_type;        // 动作类型，如 "wait", "rotate_drone_yaw_relative"
    std::string params_json;        // 动作参数
    std::string capture_filename;   // 如果是拍照动作，这是文件名
};
// --- Main Class Definition ---
class LLMProcessorNode {
public:
    LLMProcessorNode(ros::NodeHandle& nh);
    ~LLMProcessorNode();

private:
    // --- ROS Core ---
    ros::NodeHandle nh_;

    // --- Configuration ---
    std::string api_key_;
    std::string system_prompt_content_;
    std::string image_save_base_path_;
    std::string chunk_buffer_; // 用于缓存不完整的流数据块
    const size_t MAX_CONVERSATION_TURNS = 5;

    // --- Central State ---
    json conversation_history_;
    std::map<std::string, POIInfoData> _pois;
    std::vector<std::string> drone_ids_;
    std::map<std::string, DroneContext> drone_contexts_;
    std::mutex contexts_map_mutex_;

    // --- ROS Interfaces ---
    ros::Subscriber central_nlp_command_sub_;
    ros::Publisher central_nlp_feedback_pub_;
    ros::Publisher reasoning_pub_; // ++ 新增：专门用于发布思考过程的Publisher
    std::vector<ros::Subscriber> drone_pose_subs_;
    std::vector<ros::Subscriber> drone_status_subs_;
    std::vector<ros::Subscriber> action_feedback_subs_;
    std::map<std::string, ros::ServiceClient> execute_action_clients_;
    std::map<std::string, ros::Publisher> llm_goal_pubs_;
    ros::ServiceClient describe_scene_client_;
    ros::ServiceClient capture_image_client_;

    // --- Threading & Asynchronous Logic ---
    std::map<std::string, std::thread> execution_threads_;
    std::thread processing_thread_;
    std::mutex command_mutex_;
    bool needs_replan_ = false;
    std::string latest_user_command_;
    std::string last_replan_reason_;
    std::string accumulated_text_response_; 
    
    // --- LLM Stream Handling ---
    std::string accumulated_reasoning_response_;
    json accumulated_tool_calls_;

    // --- Core Methods ---
    void processingLoop();
    void executionLoop(const std::string& drone_id);
    void centralNlpCommandCallback(const std_msgs::String::ConstPtr& msg);
    void dronePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, const std::string& drone_id);
    void droneStatusCallback(const std_msgs::String::ConstPtr& msg, const std::string& drone_id);
    void actionFeedbackCallback(const std_msgs::String::ConstPtr& msg, const std::string& drone_id);
    void handle_task_completion(const std::string& drone_id);
    
    // --- Tool & Task Management ---
    json getFleetToolDefinitions();
    void dispatchToolCalls(const json& tool_calls, ReplanMode mode);
    std::string executeTool(const json& tool_call, const std::string& drone_id);
    std::string send_simple_command(const std::string& drone_id, const std::string& action_type, const std::string& params_json = "{}");
    std::string go_to_waypoint_action(double x, double y, double z, const std::string& drone_id);
    std::string return_to_launch_action(const std::string& drone_id);
    std::string cancel_all_tasks_action(const std::string& drone_id);

    // --- Macro Task: Visual Inspection ---
    void start_visual_inspection_macro(const std::string& target_name, const std::string& drone_id);
NextMacroAction advance_visual_inspection_macro(const std::string& drone_id);

    // --- Helper Methods ---
    std::string getSwarmStateAsText();
    void trimConversationHistory(); 
    void publishFeedback(const std::string& text);
    void publishReasoning(const std::string& text);
    bool streamCallback(const std::string& chunk);
    bool capture_and_get_image(const std::string& dir, const std::string& filename, sensor_msgs::Image& out_image, const std::string& drone_id);
    double calculate_yaw_to_target(const std::string& target_name, const geometry_msgs::PoseStamped& drone_pose);
    void publishGoal(const geometry_msgs::PoseStamped& goal_pose, const std::string& drone_id);
    tf2::Vector3 transformRosToUnity(const tf2::Vector3& ros_vec);
    
    std::thread llm_request_thread_; // 用于执行阻塞的 LLM API 请求
    std::atomic<bool> interrupt_llm_request_{false}; // 用于中断请求的原子标志
    std::mutex llm_thread_mutex_; // 保护 llm_request_thread_ 的创建

    // 添加一个新函数来处理停止命令
    void stopCommandCallback(const std_msgs::Empty::ConstPtr& msg);

    // 将原来的网络请求逻辑封装成一个新函数
    void executeLlmRequest(std::string command_to_process);

    // ROS Subscriber for the stop command
    ros::Subscriber stop_command_sub_;    
    std::atomic<bool> tool_call_started_; // ++ 添加这一行
};

#endif // LLM_PROCESSOR_NODE_H
