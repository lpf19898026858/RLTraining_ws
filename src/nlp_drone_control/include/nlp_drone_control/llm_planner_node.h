// include/nlp_drone_control/llm_planner_node.h
#pragma once

#include <string>
#include <map>
#include <thread>
#include <mutex>
#include <atomic>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#include <cpr/cpr.h>
#include <nlohmann/json.hpp>
#include <tf/transform_listener.h>
#include "nlp_drone_control/common_types.h"
#include "nlp_drone_control/Plan.h"
#include "nlp_drone_control/Action.h"
#include <uav_scheduler/ComputeAssignment.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sstream>
#include <iterator>
#include <boost/archive/iterators/base64_from_binary.hpp>
#include <boost/archive/iterators/transform_width.hpp>

#include "poi_state_server/ListPOIs.h"
#include <unordered_set>
#include <unordered_map>

using json = nlohmann::json;
// === forward declarations for helper functions ===
static std::map<std::string, std::vector<std::string>>
parseManualAssignments(const std::string& text);

static std::map<std::string, std::vector<std::string>>
assignTasksByDistance(const std::vector<std::string>& drones,
                      const std::vector<std::string>& pois);

static std::string
buildDirectPlanText(const std::map<std::string, std::vector<std::string>>& assignments);

/**
 * @brief Planner node: Handles LLM reasoning and tool call translation,
 * publishes Plan messages for Executor. Keeps conversation history and user command state.
 */
class LLMPlannerNode {
public:
    explicit LLMPlannerNode(ros::NodeHandle& nh);
    ~LLMPlannerNode();

private:
    enum class RequestMode { REASONING_ONLY, TOOL_CALLS };

    // === ROS Core ===
    ros::NodeHandle nh_;

    // === Configuration ===
    std::string api_key_;
    std::string system_prompt_content_;
    const size_t MAX_CONVERSATION_TURNS = 5;

    // === Conversation State ===
    json conversation_history_;
    std::string last_user_command_;
    std::string last_reasoning_text_;
    std::string last_plan_text_;
    std::atomic<bool> has_cached_plan_{false};
std::string chosen_algo_;

    // === LLM Streaming State ===
    std::string stream_buffer_;          ///< SSE data buffer
    std::mutex stream_buffer_mutex_;
    std::string accumulated_reasoning_response_;
    json accumulated_tool_calls_;
    std::map<int, json> in_progress_tool_calls_;
    std::string chunk_buffer_;          ///< buffer for incomplete chunks (original)

    // === Planner Threading ===
    std::thread llm_request_thread_;
    std::thread processing_thread_;     ///< background processing thread (kept)
    std::mutex llm_thread_mutex_;
    std::mutex command_mutex_;          ///< protect latest_user_command_ etc.
    std::atomic<bool> interrupt_llm_request_{false};
    std::atomic<bool> tool_call_started_{false};
    RequestMode current_mode = RequestMode::REASONING_ONLY;

    // === Command Handling State ===
    bool needs_replan_ = false;
    std::string latest_user_command_;
    std::string last_replan_reason_;
    std::string accumulated_text_response_; ///< kept from original node

    // === ROS Interfaces ===
    ros::Subscriber central_nlp_command_sub_;
    ros::Subscriber rereasoning_sub_;
    ros::Subscriber run_sub_;
    ros::Subscriber stop_command_sub_;
    ros::Subscriber context_text_sub_;
    ros::Subscriber context_image_sub_;

    ros::Publisher plan_pub_;
    ros::Publisher central_nlp_feedback_pub_;
    ros::Publisher reasoning_pub_;

    std::string latest_context_text_;
    sensor_msgs::Image latest_context_image_;

ros::ServiceClient scheduler_client_;
// 类成员变量（在 llm_planner_node.h 中添加）
std::vector<std::string> configured_drones_;
ros::ServiceClient list_pois_client_;
std::vector<std::string> requested_pois_;

    // === Callbacks ===
    void centralNlpCommandCallback(const std_msgs::String::ConstPtr& msg);
    void rereasoningCallback(const std_msgs::Empty::ConstPtr& msg);
    void runCallback(const std_msgs::Empty::ConstPtr& msg);
    void stopCommandCallback(const std_msgs::Empty::ConstPtr& msg);
    void contextTextCallback(const std_msgs::String::ConstPtr& msg);
    void contextImageCallback(const sensor_msgs::Image::ConstPtr& msg);

    // === LLM Request / Streaming ===
    void executeLlmRequest(std::string command_to_process, RequestMode mode);
    bool streamCallbackForGemini(const std::string& chunk);

    // === Utilities ===
    void publishFeedback(const std::string& text);
    void publishReasoning(const std::string& text);
    void trimConversationHistory();
    std::string extractPlanFromReasoning(const std::string& md);
    json getGeminiToolDefinitions();
    void processingLoop(); ///< preserved from original (optional replan loop)
     std::vector<nlp_drone_control::Action> convertTextPlanToActions(const std::string& plan_text);
std::string extractDecisionFromReasoning(const std::string& md);
std::vector<std::string> extractPOIsFromPlan(const std::string& plan_text);
};

