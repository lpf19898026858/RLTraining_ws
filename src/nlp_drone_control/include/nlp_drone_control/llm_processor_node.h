#ifndef LLM_PROCESSOR_NODE_H
#define LLM_PROCESSOR_NODE_H

// Standard C++ Library Includes
#include <string>       // For std::string
#include <vector>       // For std::vector
#include <sstream>      // For std::stringstream (used in streamCallback)
#include <string_view>  // For std::string_view (used in cpr::WriteCallback)

// ROS Includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
//#include <std_msgs/Bool.h>
#include <fstream> // For file operations
#include <sstream> // For string streams
#include <ros/package.h>

// External Library Includes
#include <cpr/cpr.h>        // For cpr library functionality
#include <nlohmann/json.hpp> // For JSON parsing and manipulation
#include "nlp_drone_control/magic_enum.hpp"

// Add these includes for XmlRpcValue if you choose to load POIs from YAML
#include <XmlRpcValue.h>
#include <XmlRpcException.h>
#include <map> // For POIs
#include <cmath> // For M_PI (defines mathematical constants like PI)
#include "vlm_service/DescribeScene.h"
#include <tf/transform_datatypes.h> // For tf::createQuaternionMsgFromYaw
#include <tf/transform_listener.h> // 确保包含了tf库
#include <tf/tf.h> // for quaternion operations (roll, pitch, yaw)
#include <thread> 
#include <mutex>  
#include "nlp_drone_control/ExecuteDroneAction.h" 
// Alias for nlohmann::json for convenience
using json = nlohmann::json;
#include <deque> 
#include <std_msgs/String.h> 
#include "vlm_service/CaptureImage.h" 
#include "nlp_drone_control/llm_processor_node.h"
#include <tf2/LinearMath/Quaternion.h> // For quaternion operations
#include <tf2/LinearMath/Matrix3x3.h>  // For converting quaternion to RPY
#include <cmath> // For atan2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
#include <filesystem> 
#include <boost/bind.hpp>
#include <future> // 用于 std::async

enum DroneState {
    GROUNDED,
    TAKING_OFF,
    HOVERING,
    FLYING,
    LANDING,
    PERFORMING_ACTION,
    UNKNOWN // 新增一个未知状态
};

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

//用于存储每个无人机完整状态的、可复用的数据结构
struct DroneContext {
    std::string id;
    
    // 状态
    std::string status_str = "UNKNOWN";
    DroneState state_enum = UNKNOWN;
    geometry_msgs::PoseStamped pose;
    geometry_msgs::Point launch_position;
    bool has_recorded_launch_position = false;
    ros::Time navigation_start_time_; // <-- 新增：记录导航开始时间
    
    // 每个无人机专属的任务队列和状态 
    std::deque<json> tool_call_queue;
    bool is_task_executing = false; // 标志这架无人机是否正在执行一个来自队列的任务
    bool action_completed = true;   // 动作完成标志
    std::mutex context_data_mutex; // 用于保护这个结构体内部数据的锁（尤其是队列）
};


class LLMProcessorNode {
public:
    LLMProcessorNode(ros::NodeHandle& nh);
    ~LLMProcessorNode();

private:
    //全局共享资源
    ros::NodeHandle nh_;
    std::string api_key_;
    std::string system_prompt_content_;
    json conversation_history_;
    std::map<std::string, POIInfoData> _pois;
    
    // --- 机队管理核心数据结构 ---
    std::vector<std::string> drone_ids_; // 从参数服务器加载的所有无人机ID
    std::map<std::string, DroneContext> drone_contexts_;
    std::mutex contexts_map_mutex_; // 保护整个 map 结构的锁

    // --- ROS 接口 ---
    // 中央接口，用于与用户交互
    ros::Subscriber central_nlp_command_sub_;
    ros::Publisher central_nlp_feedback_pub_;
    
    // 分布式接口，用于监控和控制每个无人机
    std::vector<ros::Subscriber> drone_pose_subs_;
    std::vector<ros::Subscriber> drone_status_subs_;
    std::vector<ros::Subscriber> action_feedback_subs_;
    
    std::map<std::string, ros::ServiceClient> execute_action_clients_;
    //std::map<std::string, ros::ServiceClient> capture_image_clients_;
    std::map<std::string, ros::Publisher> llm_goal_pubs_; // 每个无人机专属的A*目标发布者
    ros::ServiceClient describe_scene_client_;
    ros::ServiceClient capture_image_client_;
    // 为每个无人机启动一个执行线程 
    std::map<std::string, std::thread> execution_threads_;

    // --- 异步处理与LLM交互 ---
    std::thread processing_thread_;
    std::mutex command_mutex_;
    std::string last_command_;
    bool new_command_received_ = false;

    // LLM流式响应处理
    std::string accumulated_reasoning_response_;
    std::string accumulated_text_response_;
    json accumulated_tool_calls_;

    // --- 核心方法 ---
    void processingLoop();
    void centralNlpCommandCallback(const std_msgs::String::ConstPtr& msg);
    
    // --- 针对特定无人机的回调方法 ---
    void dronePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, const std::string& drone_id);
    void droneStatusCallback(const std_msgs::String::ConstPtr& msg, const std::string& drone_id);
    void actionFeedbackCallback(const std_msgs::String::ConstPtr& msg, const std::string& drone_id);

    // --- 任务规划与分发方法 ---
    json getFleetToolDefinitions();
    void dispatchToolCalls(const json& tool_calls);
    
    //状态驱动的任务执行器
    // 这个函数会在一个独立的线程中为每个无人机运行
    void executionLoop(const std::string& drone_id);
    std::string executeTool(const json& tool_call, const std::string& drone_id); // 执行单个工具
    
    std::string takeoff_action(const std::string& drone_id);
    std::string land_action(const std::string& drone_id);
    std::string go_to_waypoint_action(double x, double y, double z, const std::string& drone_id);
    std::string wait_action(double duration, const std::string& drone_id);
    std::string perform_visual_inspection_action(const std::string& target_name, const std::string& drone_id);
std::string return_to_launch_action(const std::string& drone_id);
    // --- 复用并修改后的辅助函数 ---
    void publishGoal(const geometry_msgs::PoseStamped& goal_pose, const std::string& drone_id);
    bool rotate_yaw_action(double degrees, const std::string& drone_id);
    bool backup_action(double distance, const std::string& drone_id);
    std::string adjust_camera_action(double pitch_angle, const std::string& drone_id);
    bool capture_and_get_image(const std::string& dir, const std::string& filename, sensor_msgs::Image& out_image, const std::string& drone_id);
    tf2::Vector3 transformRosToUnity(const tf2::Vector3& ros_vec);
bool execute_and_wait_for_completion(const std::string& drone_id, const std::string& action_type, const std::string& params_json, const std::string& step_name, double timeout_sec = 20.0);
    const size_t MAX_CONVERSATION_TURNS = 5; // 保留系统+5轮用户/助手对话

    // --- 辅助方法 ---
    void trimConversationHistory(); 
    void publishFeedback(const std::string& text);
    bool streamCallback(const std::string& chunk);
    
    std::string image_save_base_path_;
};

#endif // LLM_PROCESSOR_NODE_H
