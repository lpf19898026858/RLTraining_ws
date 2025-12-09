// include/nlp_drone_control/task_executor_node.h
#pragma once

#include <string>
#include <vector>
#include <map>
#include <deque>
#include <thread>
#include <mutex>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <cfloat>
#include <atomic>
#include <regex>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <XmlRpcValue.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include "nlp_drone_control/ExecuteDroneAction.h"
#include "vlm_service/DescribeScene.h"
#include "vlm_service/CaptureImage.h"

#include "nlp_drone_control/Plan.h"
#include "nlp_drone_control/Action.h"
#include "nlp_drone_control/common_types.h"
#include "poi_state_server/ListPOIs.h"
#include "poi_state_server/GetPOIInfo.h"

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

/**
 * @brief Executor node: Maintains swarm state, executes queued tool calls,
 * and publishes context text periodically for the Planner.
 */
class TaskExecutorNode {
public:
    explicit TaskExecutorNode(ros::NodeHandle& nh);
    ~TaskExecutorNode();

private:
    ros::NodeHandle nh_;
    std::string image_save_base_path_;

  // ========== NEW: POI state server clients ==========
  ros::ServiceClient get_poi_info_client_;
  ros::ServiceClient list_pois_client_;
  
  ros::Publisher fleet_context_image_pub_;
cv::Mat overview_image_;
  
    std::map<std::string, POIInfoData> _pois;
    std::vector<std::string> drone_ids_;
    std::map<std::string, DroneContext> drone_contexts_;
    std::mutex contexts_map_mutex_;

    // ROS Interfaces
    ros::Subscriber planner_tool_calls_sub_;
    ros::Subscriber stop_command_sub_;
    std::vector<ros::Subscriber> drone_pose_subs_;
    std::vector<ros::Subscriber> drone_status_subs_;
    std::vector<ros::Subscriber> action_feedback_subs_;

    ros::Publisher fleet_context_pub_;
    ros::Publisher central_nlp_feedback_pub_;
    ros::Publisher reasoning_pub_;
    std::map<std::string, ros::Publisher> llm_goal_pubs_;

    std::map<std::string, ros::ServiceClient> execute_action_clients_;
    ros::ServiceClient describe_scene_client_;
    ros::ServiceClient capture_image_client_;

    ros::Timer context_timer_;
    std::map<std::string, std::thread> execution_threads_;

    // === Callbacks ===
    void plannerPlanCallback(const nlp_drone_control::Plan::ConstPtr& msg);
    void stopCommandCallback(const std_msgs::Empty::ConstPtr& msg);
    void dronePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, const std::string& drone_id);
    void droneStatusCallback(const std_msgs::String::ConstPtr& msg, const std::string& drone_id);
    void actionFeedbackCallback(const std_msgs::String::ConstPtr& msg, const std::string& drone_id);
    void onContextTimer(const ros::TimerEvent&);

    // === Core Logic ===
    void executionLoop(const std::string& drone_id);
    void dispatchToolCalls(const json& tool_calls, ReplanMode mode);
    std::string executeTool(const json& tool_call, const std::string& drone_id);
    std::string send_simple_command(const std::string& drone_id, const std::string& action_type, const std::string& params_json = "{}");

    // === Action Helpers ===
    std::string go_to_waypoint_action(double x, double y, double z, const std::string& drone_id);
    std::string return_to_launch_action(const std::string& drone_id);
    std::string cancel_all_tasks_action(const std::string& drone_id);

    // === Macro Tasks ===
    void start_visual_inspection_macro(const std::string& target_name, const std::string& drone_id);
    NextMacroAction advance_visual_inspection_macro(const std::string& drone_id);
    void handle_task_completion(const std::string& drone_id);

    // === Utility Functions ===
    std::string getSwarmStateAsText();
    void publishFeedback(const std::string& text);
    bool capture_and_get_image(const std::string& dir, const std::string& filename, sensor_msgs::Image& out_image, const std::string& drone_id);
    double calculate_yaw_to_target(const std::string& target_name, const geometry_msgs::PoseStamped& drone_pose);
    void publishGoal(const geometry_msgs::PoseStamped& goal_pose, const std::string& drone_id);
    tf2::Vector3 transformRosToUnity(const tf2::Vector3& ros_vec);
};

