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
#include <thread> // 包含头文件
#include <mutex>  // 包含头文件
#include "nlp_drone_control/ExecuteDroneAction.h" 
// Alias for nlohmann::json for convenience
using json = nlohmann::json;
#include <deque> // 使用 deque 作为队列
#include <std_msgs/String.h> 
#include "vlm_service/CaptureImage.h" // <<< 引用服务头文件
#include "nlp_drone_control/llm_processor_node.h"
#include <tf2/LinearMath/Quaternion.h> // For quaternion operations
#include <tf2/LinearMath/Matrix3x3.h>  // For converting quaternion to RPY
#include <cmath> // For atan2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
    //无人机状态枚举
enum DroneState {
    GROUNDED,   // 在地面
    TAKING_OFF, // 正在起飞
    HOVERING,   // 在空中悬停
    FLYING,     // 正在飞行（但不是起飞或降落过程中）
    LANDING,     // 正在降落
    PERFORMING_ACTION   //look_around,describe_scene
};

struct NamedPoint {
    std::string name;
    geometry_msgs::Point point;
};

// 用于表示一个简化的 2D/3D 边界框
struct SimplifiedBoundary {
    geometry_msgs::Point min;
    geometry_msgs::Point max;
};

// 用于存储每个兴趣点（POI）的完整信息的主数据结构
struct POIInfoData {
    std::string name;
    std::string type;
    std::string description;
    geometry_msgs::Point position; // POI 的参考中心点

    // 一个包含多个潜在、安全接近点的列表，供 LLM 选择
    std::vector<NamedPoint> candidate_points;

    // 用于表示简化障碍物边界的标志和数据
    bool has_simplified_boundary = false;
    SimplifiedBoundary simplified_boundary;
};
class LLMProcessorNode {
public:
    LLMProcessorNode(ros::NodeHandle& nh);
    ~LLMProcessorNode();

private:
    ros::NodeHandle nh_; // 保存 NodeHandle
    ros::Publisher nlp_feedback_pub_;
    ros::Subscriber _current_pose_sub_; 
    ros::Subscriber nlp_command_sub_;
    ros::Publisher llm_goal_pub_;     // ADDED: This will publish the final goal to the A* planner
    ros::Subscriber _drone_status_sub_; 
    ros::Publisher direct_unity_pub_; 
     ros::Subscriber _action_feedback_sub; 

    DroneState _drone_state; // 当前无人机状态
    geometry_msgs::PoseStamped _current_pose; // 存储无人机当前姿态
    std::string system_prompt_content_;
    std::string _drone_status;          // 存储无人机状态 ("IDLE" 或 "NAVIGATING")
    
    void droneStatusCallback(const std_msgs::String::ConstPtr& msg); 
    
    std::string api_key_; 
    std::string accumulated_reasoning_response_;
    std::string accumulated_text_response_; // Not directly used for publishing anymore, but good for debugging
    json accumulated_tool_calls_;           // Stores parsed tool calls from LLM stream
json conversation_history_; // 用于存储对话历史

    std::map<std::string, POIInfoData> _pois;
    ros::ServiceClient describe_scene_client_; // 添加Service Client成员
    ros::ServiceClient execute_action_client_; // 用于调用Unity中的动作服务
    ros::ServiceClient capture_image_client_;
    
    geometry_msgs::Point _launch_position; // 存储起飞点的三维坐标
    bool _has_recorded_launch_position;    // 标志位，防止重复记录
    
    bool streamCallback(const std::string& chunk);
    void nlpCommandCallback(const std_msgs::String::ConstPtr& msg);
    json getToolDefinitions();
    void publishFeedback(const std::string& text); 
    void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void publishGoal(const geometry_msgs::PoseStamped& goal_pose, const std::string& action_name);

    std::string takeoff_action();
    std::string land_action();
    std::string go_to_waypoint_action(double x, double y, double z);
    
    std::thread _processing_thread;
    std::mutex _command_mutex;
    std::string _last_command;
    bool _new_command_received = false;
    
    //任务队列和状态管理 ---
    std::deque<json> _tool_call_queue; // 存储待执行的工具调用
    bool _is_task_executing;           // 标志位：当前是否有任务正在执行
    std::mutex _feedback_mutex;           // 用于线程安全的互斥锁
    bool _action_completed;               // 标志位
    std::mutex _pose_mutex;   
    std::mutex _status_mutex; 
    
    void executeNextToolInQueue();
    std::string executeTool(const json& tool_call);
    
    std::string perform_visual_inspection_action(const std::string& target_name); 
bool rotate_yaw_action(double degrees); 
std::string inspection_sequence_action();
    std::string adjust_camera_action(double pitch_angle);
    bool capture_and_get_image(const std::string& dir, const std::string& filename, sensor_msgs::Image& out_image);
    void actionFeedbackCallback(const std_msgs::String::ConstPtr& msg); 
    bool backup_action(double distance);
    std::string wait_action(double duration);
    void processingLoop(); 
    tf2::Vector3 transformRosToUnity(const tf2::Vector3& ros_vec);
    
    std::string initial_poi_name;
    //ROS Interface Configuration ---
    // Topic Names
    std::string nlp_feedback_topic_;
    std::string nlp_command_topic_;
    std::string drone_pose_topic_;
    std::string drone_status_topic_;
    std::string llm_goal_topic_;
    std::string drone_target_topic_; // For Unity
    std::string action_feedback_topic_;

    // Service Names
    std::string describe_scene_service_;
    std::string execute_action_service_;
    std::string capture_image_service_;
    
    // Parameter Names
    std::string api_key_param_;
    std::string system_prompt_file_param_;
    std::string points_of_interest_param_;    
};

#endif // LLM_PROCESSOR_NODE_H
