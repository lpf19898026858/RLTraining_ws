#include "nlp_drone_control/llm_processor_node.h"

LLMProcessorNode::LLMProcessorNode(ros::NodeHandle& nh):nh_(nh){
// 私有节点句柄，用于读取本节点专属的参数
ros::NodeHandle pnh("~");
ROS_INFO("LLM Fleet Commander Node (Central Brain) is initializing...");

if (!nh_.getParam("/deepseek_api_key", api_key_) || api_key_.empty()) {
    ROS_WARN("Global parameter '/deepseek_api_key' not set!");
}

std::string prompt_file_path;
if (!nh_.getParam("/system_prompt_file", prompt_file_path)) {
    ROS_ERROR("Global parameter '/system_prompt_file' not found.");
    system_prompt_content_ = "You are an expert drone control assistant.";
} else {
    std::ifstream prompt_file(prompt_file_path);
    if (prompt_file.is_open()) {
        std::stringstream buffer;
        buffer << prompt_file.rdbuf();
        system_prompt_content_ = buffer.str();
        prompt_file.close();
        ROS_INFO_STREAM("Successfully loaded system prompt from: " << prompt_file_path);
    } else {
        ROS_ERROR_STREAM("Failed to open system prompt file: " << prompt_file_path);
        system_prompt_content_ = "You are an expert drone control assistant."; // Fallback
    }
}
    // 2. 加载无人机ID列表
pnh.getParam("drone_ids", drone_ids_);
if (drone_ids_.empty()) {
    ROS_FATAL("Private parameter 'drone_ids' is not set. Commander needs to know which drones to manage.");
    ros::shutdown();
    return;
}
pnh.param<std::string>("image_save_base_path", image_save_base_path_, "/tmp/drone_images");
XmlRpc::XmlRpcValue poi_list_param;
if (nh_.getParam("/points_of_interest", poi_list_param)) {
//ROS_INFO("Loading Points of Interest from YAML (Hybrid Strategy)...");

if (poi_list_param.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    for (int i = 0; i < poi_list_param.size(); ++i) {
        XmlRpc::XmlRpcValue& poi_entry = poi_list_param[i];

        if (poi_entry.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
            POIInfoData current_poi_data;

            // --- Extract common fields (name, description, type) ---
            if (poi_entry.hasMember("name")) { current_poi_data.name = static_cast<std::string>(poi_entry["name"]); }
            if (poi_entry.hasMember("description")) { current_poi_data.description = static_cast<std::string>(poi_entry["description"]); }
            if (poi_entry.hasMember("type")) { current_poi_data.type = static_cast<std::string>(poi_entry["type"]); }
            
            // --- Extract reference position ---
            if (poi_entry.hasMember("position") && poi_entry["position"].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
                XmlRpc::XmlRpcValue& pos = poi_entry["position"];
                if (pos.hasMember("x")) { if (pos["x"].getType() == XmlRpc::XmlRpcValue::TypeDouble) current_poi_data.position.x = pos["x"]; else current_poi_data.position.x = static_cast<double>(static_cast<int>(pos["x"])); }
                if (pos.hasMember("y")) { if (pos["y"].getType() == XmlRpc::XmlRpcValue::TypeDouble) current_poi_data.position.y = pos["y"]; else current_poi_data.position.y = static_cast<double>(static_cast<int>(pos["y"])); }
                if (pos.hasMember("z")) { if (pos["z"].getType() == XmlRpc::XmlRpcValue::TypeDouble) current_poi_data.position.z = pos["z"]; else current_poi_data.position.z = static_cast<double>(static_cast<int>(pos["z"])); }
            }

            // --- Extract candidate_interaction_points (list of structs) ---
            if (poi_entry.hasMember("candidate_interaction_points") && poi_entry["candidate_interaction_points"].getType() == XmlRpc::XmlRpcValue::TypeArray) {
                XmlRpc::XmlRpcValue& candidates_list = poi_entry["candidate_interaction_points"];
                for (int j = 0; j < candidates_list.size(); ++j) {
                    XmlRpc::XmlRpcValue& candidate_entry = candidates_list[j];
                    if (candidate_entry.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
                        NamedPoint named_pt;
                        if (candidate_entry.hasMember("name")) { named_pt.name = static_cast<std::string>(candidate_entry["name"]); }
                        
                        if (candidate_entry.hasMember("point") && candidate_entry["point"].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
                            XmlRpc::XmlRpcValue& pt_coords = candidate_entry["point"];
                            if (pt_coords.hasMember("x")) { if (pt_coords["x"].getType() == XmlRpc::XmlRpcValue::TypeDouble) named_pt.point.x = pt_coords["x"]; else named_pt.point.x = static_cast<double>(static_cast<int>(pt_coords["x"])); }
                            if (pt_coords.hasMember("y")) { if (pt_coords["y"].getType() == XmlRpc::XmlRpcValue::TypeDouble) named_pt.point.y = pt_coords["y"]; else named_pt.point.y = static_cast<double>(static_cast<int>(pt_coords["y"])); }
                            if (pt_coords.hasMember("z")) { if (pt_coords["z"].getType() == XmlRpc::XmlRpcValue::TypeDouble) named_pt.point.z = pt_coords["z"]; else named_pt.point.z = static_cast<double>(static_cast<int>(pt_coords["z"])); }
                        }
                        current_poi_data.candidate_points.push_back(named_pt);
                    }
                }
            }

            // --- Extract simplified_boundary (struct of structs) ---
            if (poi_entry.hasMember("simplified_boundary") && poi_entry["simplified_boundary"].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
                XmlRpc::XmlRpcValue& sb = poi_entry["simplified_boundary"];
                if (sb.hasMember("min") && sb["min"].getType() == XmlRpc::XmlRpcValue::TypeStruct &&
                    sb.hasMember("max") && sb["max"].getType() == XmlRpc::XmlRpcValue::TypeStruct)
                {
                    current_poi_data.has_simplified_boundary = true;
                    XmlRpc::XmlRpcValue& min_coords = sb["min"];
                    XmlRpc::XmlRpcValue& max_coords = sb["max"];

                    if (min_coords.hasMember("x")) { if (min_coords["x"].getType() == XmlRpc::XmlRpcValue::TypeDouble) current_poi_data.simplified_boundary.min.x = min_coords["x"]; else current_poi_data.simplified_boundary.min.x = static_cast<double>(static_cast<int>(min_coords["x"])); }
                    if (min_coords.hasMember("y")) { if (min_coords["y"].getType() == XmlRpc::XmlRpcValue::TypeDouble) current_poi_data.simplified_boundary.min.y = min_coords["y"]; else current_poi_data.simplified_boundary.min.y = static_cast<double>(static_cast<int>(min_coords["y"])); }
                    if (min_coords.hasMember("z")) { if (min_coords["z"].getType() == XmlRpc::XmlRpcValue::TypeDouble) current_poi_data.simplified_boundary.min.z = min_coords["z"]; else current_poi_data.simplified_boundary.min.z = static_cast<double>(static_cast<int>(min_coords["z"])); }

                    if (max_coords.hasMember("x")) { if (max_coords["x"].getType() == XmlRpc::XmlRpcValue::TypeDouble) current_poi_data.simplified_boundary.max.x = max_coords["x"]; else current_poi_data.simplified_boundary.max.x = static_cast<double>(static_cast<int>(max_coords["x"])); }
                    if (max_coords.hasMember("y")) { if (max_coords["y"].getType() == XmlRpc::XmlRpcValue::TypeDouble) current_poi_data.simplified_boundary.max.y = max_coords["y"]; else current_poi_data.simplified_boundary.max.y = static_cast<double>(static_cast<int>(max_coords["y"])); }
                    if (max_coords.hasMember("z")) { if (max_coords["z"].getType() == XmlRpc::XmlRpcValue::TypeDouble) current_poi_data.simplified_boundary.max.z = max_coords["z"]; else current_poi_data.simplified_boundary.max.z = static_cast<double>(static_cast<int>(max_coords["z"])); }
                }
            }
            
            // Store the fully parsed data
            _pois[current_poi_data.name] = current_poi_data;
            //ROS_INFO_STREAM("Loaded POI: " << current_poi_data.name);

        } else {
            ROS_WARN_STREAM("Entry at index " << i << " in 'points_of_interest' is not a dictionary/struct. Skipping.");
        }
    }
} else {
    ROS_WARN("Parameter 'points_of_interest' is not a list/array type. Check your YAML.");
}
} else {
ROS_WARN("Global parameter '/points_of_interest' not found.");
}

// 4. 为每个无人机创建ROS接口并初始化状态
for (const std::string& drone_id : drone_ids_) {
    // 初始化上下文
    drone_contexts_[drone_id].id = drone_id;
    if (_pois.count(drone_id)) {
        drone_contexts_[drone_id].pose.pose.position = _pois[drone_id].position;
        drone_contexts_[drone_id].launch_position = _pois[drone_id].position;
    }

    // 创建订阅者
    drone_pose_subs_.push_back(nh_.subscribe<geometry_msgs::PoseStamped>("/" + drone_id + "/drone_pose", 1, boost::bind(&LLMProcessorNode::dronePoseCallback, this, _1, drone_id)));
    drone_status_subs_.push_back(nh_.subscribe<std_msgs::String>("/" + drone_id + "/drone_status", 1, boost::bind(&LLMProcessorNode::droneStatusCallback, this, _1, drone_id)));
    action_feedback_subs_.push_back(nh_.subscribe<std_msgs::String>("/" + drone_id + "/action_feedback", 1, boost::bind(&LLMProcessorNode::actionFeedbackCallback, this, _1, drone_id)));

    // 创建服务客户端和发布者
    execute_action_clients_[drone_id] = nh_.serviceClient<nlp_drone_control::ExecuteDroneAction>("/" + drone_id + "/execute_drone_action");
    //capture_image_clients_[drone_id] = nh_.serviceClient<vlm_service::CaptureImage>("/" + drone_id + "/capture_image");
    llm_goal_pubs_[drone_id] = nh_.advertise<geometry_msgs::PoseStamped>("/" + drone_id + "/llm_goal", 1);
}

rereasoning_sub_ = nh_.subscribe("/nlp_rereasoning", 1, &LLMProcessorNode::rereasoningCallback, this);
run_sub_ = nh_.subscribe("/nlp_run", 1, &LLMProcessorNode::runCallback, this);

// 5. 创建中央接口
describe_scene_client_ = nh_.serviceClient<vlm_service::DescribeScene>("/vision_service/describe_scene");
central_nlp_command_sub_ = nh_.subscribe("/nlp_command", 1, &LLMProcessorNode::centralNlpCommandCallback, this);
central_nlp_feedback_pub_ = nh_.advertise<std_msgs::String>("/nlp_feedback", 10);
reasoning_pub_ = nh_.advertise<std_msgs::String>("/nlp_reasoning", 10);
capture_image_client_ = nh_.serviceClient<vlm_service::CaptureImage>("/vision_service/capture_image");
stop_command_sub_ = nh_.subscribe("/nlp_stop", 1, &LLMProcessorNode::stopCommandCallback, this);

// 6. 初始化全局对话历史
conversation_history_ = json::array();
conversation_history_.push_back({{"role", "system"}, {"content", ""}});

// 7. 启动规划和执行线程
for (const std::string& drone_id : drone_ids_) {
    execution_threads_[drone_id] = std::thread(&LLMProcessorNode::executionLoop, this, drone_id);
}
processing_thread_ = std::thread(&LLMProcessorNode::processingLoop, this);
ROS_INFO("LLM Fleet Commander is ready to command the fleet.");
}

LLMProcessorNode::~LLMProcessorNode() {
if (processing_thread_.joinable()) {
processing_thread_.join(); // 等待线程结束
}
for(auto& pair : execution_threads_){
if(pair.second.joinable()){
pair.second.join();
}
}
}
void LLMProcessorNode::centralNlpCommandCallback(const std_msgs::String::ConstPtr& msg) {
std::lock_guard<std::mutex> lock(command_mutex_);
latest_user_command_ = msg->data;
needs_replan_ = true;
last_replan_reason_ = "New user command received: '" + msg->data + "'";
ROS_INFO_STREAM("Replanning triggered by new user command.");
}
void LLMProcessorNode::stopCommandCallback(const std_msgs::Empty::ConstPtr& msg) {
ROS_WARN("LLM STOP command received! Interrupting current thought process.");
publishFeedback("User interrupted. LLM is stopping...");

// 设置中断标志
interrupt_llm_request_.store(true);

// 清理可能存在的旧规划请求
std::lock_guard<std::mutex> lock(command_mutex_);
needs_replan_ = false;
latest_user_command_.clear();
last_replan_reason_.clear();
}
void LLMProcessorNode::rereasoningCallback(const std_msgs::Empty::ConstPtr& msg) {
    ROS_INFO("Rereasoning command received from UI.");
    publishFeedback("User requested re-reasoning. Restarting reasoning phase...");

    // 先打断旧流
    interrupt_llm_request_.store(true);

    std::lock_guard<std::mutex> lock(llm_thread_mutex_);
    if (llm_request_thread_.joinable()) {
        llm_request_thread_.join();
    }
    interrupt_llm_request_.store(false); // 准备新请求

    llm_request_thread_ = std::thread(
        &LLMProcessorNode::executeLlmRequest,
        this,
        last_user_command_,              // 复用上一轮的prompt（见第4点的说明）
        RequestMode::REASONING_ONLY
    );
}

void LLMProcessorNode::runCallback(const std_msgs::Empty::ConstPtr& msg) {
    ROS_INFO("Run command received from UI.");
    publishFeedback("User confirmed execution. Running tool calls...");

    // 先打断旧流
    interrupt_llm_request_.store(true);

    std::lock_guard<std::mutex> lock(llm_thread_mutex_);
    if (llm_request_thread_.joinable()) {
        llm_request_thread_.join();
    }
    interrupt_llm_request_.store(false); // 准备新请求

    llm_request_thread_ = std::thread(
        &LLMProcessorNode::executeLlmRequest,
        this,
        last_user_command_,              // 复用上一轮的prompt（见第4点的说明）
        RequestMode::TOOL_CALLS
    );
}


void LLMProcessorNode::dronePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, const std::string& drone_id) {
std::lock_guard<std::mutex> lock(contexts_map_mutex_);
if (drone_contexts_.count(drone_id)) drone_contexts_.at(drone_id).pose = *msg;
}

void LLMProcessorNode::droneStatusCallback(const std_msgs::String::ConstPtr& msg, const std::string& drone_id) {
bool should_handle_completion = false;

// 作用域限定的大括号，用于自动管理互斥锁的生命周期
{
    std::lock_guard<std::mutex> lock(contexts_map_mutex_);
    
    // 步骤 1: 检查 drone_id 是否有效，如果无效则直接返回
    if (!drone_contexts_.count(drone_id)) {
        ROS_WARN_STREAM("Received status from unknown drone_id: " << drone_id);
        return;
    }

    auto& context = drone_contexts_.at(drone_id);
    DroneState old_state = context.state_enum;
    
    // 步骤 2: 更新无人机的状态
    context.status_str = msg->data;
    if (context.status_str == "IDLE_ON_GROUND") context.state_enum = GROUNDED;
    else if (context.status_str == "NAVIGATING") context.state_enum = FLYING;
    else if (context.status_str == "HOVERING" || context.status_str == "IDLE_IN_AIR") context.state_enum = HOVERING;
    else if (context.status_str == "PERFORMING_ACTION") context.state_enum = PERFORMING_ACTION;
    else if (context.status_str == "TAKING_OFF") context.state_enum = TAKING_OFF;
    else if (context.status_str == "LANDING") context.state_enum = LANDING;
    else context.state_enum = UNKNOWN;
    
    DroneState new_state = context.state_enum;
    
    // 步骤 3: 判断是否发生了“任务完成”的状态转换
    if ((old_state == FLYING && new_state == HOVERING) ||
        (old_state == TAKING_OFF && new_state == HOVERING) ||
        (old_state == LANDING && new_state == GROUNDED) ||
        (old_state == PERFORMING_ACTION && new_state == HOVERING)) 
    {
        // 如果任务完成，设置一个标志位。我们不在锁内调用处理函数。
        should_handle_completion = true;
    }
} // 互斥锁 lock 在这里被自动释放

// 步骤 4: 在锁之外，安全地调用处理函数
if (should_handle_completion) {
    handle_task_completion(drone_id);
}
}

void LLMProcessorNode::actionFeedbackCallback(const std_msgs::String::ConstPtr& msg, const std::string& drone_id) {
if (msg->data == "ACTION_COMPLETE") {
handle_task_completion(drone_id);
}
}
std::string LLMProcessorNode::getSwarmStateAsText() {
std::string dynamic_info_str;

// --- Part 1: 构建机队状态信息 ---
dynamic_info_str += "You have access to the following fleet information:\n--- Fleet Status ---\n";
{
    std::lock_guard<std::mutex> lock(contexts_map_mutex_);
    if (drone_ids_.empty()) {
        dynamic_info_str += "No drones are currently available.\n";
    } else {
        for (const auto& id : drone_ids_) {
            // 使用 .at() 会在键不存在时抛出异常，更安全
            try {
                const auto& context = drone_contexts_.at(id);
                dynamic_info_str += "- Drone ID: " + context.id + "\n";
                dynamic_info_str += "  - State: " + context.status_str + "\n";
                dynamic_info_str += "  - Position: (x:" + std::to_string(context.pose.pose.position.x) + 
                                  ", y:" + std::to_string(context.pose.pose.position.y) + 
                                  ", z:" + std::to_string(context.pose.pose.position.z) + ")\n";
                if (context.has_recorded_launch_position) {
                    dynamic_info_str += "  - Launch Point: (x:" + std::to_string(context.launch_position.x) + 
                                      ", y:" + std::to_string(context.launch_position.y) + 
                                      ", z:" + std::to_string(context.launch_position.z) + ")\n";
                }
            } catch (const std::out_of_range& e) {
                ROS_ERROR_STREAM("Logic error: drone_id " << id << " not found in drone_contexts_ map during prompt generation.");
            }
        }
    }
}

// --- Part 2: 构建共享的POI信息---
dynamic_info_str += "\n--- Points of Interest (POIs) ---\n";
if (!_pois.empty()) {
    for (const auto& pair : _pois) {
        const POIInfoData& poi = pair.second;
        dynamic_info_str += "- Name: " + poi.name + "\n";
        dynamic_info_str += "  Type: " + poi.type + "\n";
        dynamic_info_str += "  Description: " + poi.description + "\n";
        dynamic_info_str += "  Reference Coordinates: (x:" + std::to_string(poi.position.x) + 
                        ", y:" + std::to_string(poi.position.y) + 
                        ", z:" + std::to_string(poi.position.z) + ")\n";

        if (!poi.candidate_points.empty()) {
            dynamic_info_str += "  **ACTIONABLE GOALS:**\n";
            for (const auto& p : poi.candidate_points) {
                dynamic_info_str += "    - " + p.name + ": (x:" + std::to_string(p.point.x) + 
                                ", y:" + std::to_string(p.point.y) + 
                                ", z:" + std::to_string(p.point.z) + ")\n";
            }
        } else {
            dynamic_info_str += "  **ACTIONABLE GOAL (Reference Point):** (x:" + std::to_string(poi.position.x) + 
                            ", y:" + std::to_string(poi.position.y) + 
                            ", z:" + std::to_string(poi.position.z) + ")\n";
        }

        if (poi.has_simplified_boundary) {
            dynamic_info_str += "  Obstacle Boundary (min_xy to max_xy): [(" +
                            std::to_string(poi.simplified_boundary.min.x) + ", " +
                            std::to_string(poi.simplified_boundary.min.y) + "), (" +
                            std::to_string(poi.simplified_boundary.max.x) + ", " +
                            std::to_string(poi.simplified_boundary.max.y) + ")]\n";
        }
        dynamic_info_str += "\n";
    }
} else {
    dynamic_info_str += "No specific Points of Interest are currently defined.\n";
}
return dynamic_info_str;
}
bool LLMProcessorNode::streamCallbackForGemini(const std::string& chunk) {
    ROS_INFO_STREAM("streamCallbackForGemini current_mode = "
        << (current_mode == RequestMode::TOOL_CALLS ? "TOOL_CALLS" : "REASONING_ONLY"));

    std::lock_guard<std::mutex> buffer_lock(stream_buffer_mutex_);
    stream_buffer_ += chunk;

    std::istringstream ss(stream_buffer_);
    std::string line;
    size_t processed_chars = 0;

    while (std::getline(ss, line)) {
        processed_chars += line.size() + 1; // +1 for '\n'

        if (line.empty()) {
            ROS_DEBUG_STREAM("[SKIP] Empty line");
            continue;
        }
        if (line.rfind("data:", 0) != 0) {
            ROS_DEBUG_STREAM("[SKIP] Non-data line: " << line);
            continue;
        }

        std::string json_str = line.substr(5);
        if (!json_str.empty() && json_str.back() == '\r') json_str.pop_back();
        ROS_INFO_STREAM("[SSE raw line] " << json_str);
        ROS_INFO_STREAM("[BUFFER SIZE after append] " << stream_buffer_.size());


        if (json_str.empty() || json_str == "[DONE]") {
            ROS_INFO_STREAM("[SKIP] Empty or [DONE] marker");
            continue;
        }

        try {
if (!json::accept(json_str)) {
    ROS_WARN_STREAM("[WAIT] Incomplete JSON, keep in buffer (len=" << json_str.size() << "): " << json_str.substr(0,200) << "...");
    // ⚠️ 不要加 processed_chars，不要 erase，直接 return true
    return true;
}
ROS_INFO_STREAM("[BUFFER before parse] size=" << stream_buffer_.size());

            json response_part = json::parse(json_str);
            ROS_INFO_STREAM("[DEBUG response_part] " << response_part.dump(2));

            if (!response_part.contains("candidates")) {
                ROS_WARN_STREAM("[SKIP] No candidates field");
                continue;
            }
            if (response_part["candidates"].empty()) {
                ROS_WARN_STREAM("[SKIP] candidates empty");
                continue;
            }

            for (const auto& cand : response_part["candidates"]) {
                if (!cand.contains("content")) {
                    ROS_WARN_STREAM("[SKIP] candidate without content: " << cand.dump(2));
                    continue;
                }
                const auto& content = cand["content"];
                ROS_INFO_STREAM("[DEBUG CANDIDATE CONTENT] " << content.dump(2));

                if (!content.contains("parts")) {
                    ROS_WARN_STREAM("[SKIP] content without parts: " << content.dump(2));
                    continue;
                }

                const auto& parts = content["parts"];
                ROS_INFO_STREAM("[DEBUG parts count] " << parts.size());

                for (const auto& part : parts) {
                    ROS_INFO_STREAM("[DEBUG PART RAW] " << part.dump(2));

                    if (current_mode == RequestMode::REASONING_ONLY) {
                        if (part.contains("text")) {
                            std::string text_chunk = part["text"];
                            accumulated_reasoning_response_ += text_chunk;
                            publishReasoning(text_chunk);
                            ROS_INFO_STREAM("[REASONING] appended text chunk");
                        } else {
                            ROS_WARN_STREAM("[REASONING] part without text: " << part.dump(2));
                        }
                    } 
                    else if (current_mode == RequestMode::TOOL_CALLS) {
                        ROS_INFO_STREAM("Enter TOOL_CALLS mode");

                        if (part.contains("functionCall") || part.contains("function_call")) {
                            ROS_INFO_STREAM("[FOUND FUNCTIONCALL] " << part.dump(2));
                            try {
                                const auto& function_call = part.contains("functionCall")
                                    ? part.at("functionCall")
                                    : part.at("function_call");

                                json formatted_tool_call;
                                formatted_tool_call["function"]["name"] = function_call.at("name");

                                if (function_call.contains("args")) {
                                    formatted_tool_call["function"]["arguments"] = function_call.at("args").dump();
                                } else {
                                    formatted_tool_call["function"]["arguments"] = "{}";
                                }

                                accumulated_tool_calls_.push_back(formatted_tool_call);
                                ROS_INFO_STREAM("[PUSHED tool call] " << formatted_tool_call.dump(2));
                            } catch (const std::exception& e) {
                                ROS_ERROR_STREAM("[ERROR parsing functionCall] " << e.what()
                                    << " | Raw part: " << part.dump(2));
                            }
                        } else {
                            if (part.contains("text")) {
                                ROS_WARN_STREAM("[TOOL_CALLS] Got text instead of functionCall: "
                                    << part["text"].get<std::string>());
                            } else {
                                ROS_WARN_STREAM("[TOOL_CALLS] No functionCall in part: " << part.dump(2));
                            }
                        }
                    }
                }
            }
        } catch (const json::exception& e) {
            ROS_ERROR_STREAM("[PARSE ERROR] " << e.what() << " | Raw line: " << json_str);
            continue;
        }
    }
ROS_INFO_STREAM("[BUFFER after erase] size=" << stream_buffer_.size());
    if (current_mode == RequestMode::TOOL_CALLS) {
        ROS_INFO_STREAM("[SUMMARY] Accumulated tool calls so far: " << accumulated_tool_calls_.size());
    }

    stream_buffer_.erase(0, processed_chars);
    return true;
}

void LLMProcessorNode::executeLlmRequest(std::string command_to_process, RequestMode mode)
{
    interrupt_llm_request_.store(false);
    publishFeedback("LLM is thinking...");
    ROS_INFO_STREAM("executeLlmRequest setting current_mode = " 
        << (mode == RequestMode::TOOL_CALLS ? "TOOL_CALLS" : "REASONING_ONLY"));
    current_mode = mode;

    last_user_command_ = command_to_process; // 记录用户的原始命令
    accumulated_tool_calls_ = json::array();
    in_progress_tool_calls_.clear();
    accumulated_reasoning_response_.clear();

    // 1. 构建 contents
    json gemini_contents = json::array();

    if (mode == RequestMode::REASONING_ONLY) {
        // 推理模式：把用户命令传进去
        gemini_contents.push_back({
            {"role", "user"},
            {"parts", {{{"text", command_to_process}}}}
        });
    } else { // TOOL_CALLS 模式
        if (!has_cached_plan_.load()) {
            publishFeedback("No cached plan to execute. Please run reasoning first.");
            return;
        }
    // 把 POI/机群上下文也给到译器（很关键）
    std::string ctx = getSwarmStateAsText();

    // 只喂动作序列 + 清晰映射规则
    std::string translator_user_text;
    translator_user_text.reserve(4096);
    translator_user_text += "Context:\n";
    translator_user_text += ctx;
    translator_user_text += "\n\n"
        "Translate the following action sequence plan into tool calls. "
        "Do NOT rethink or reorder. Keep it 1:1.\n\n"
        "Mapping rules:\n"
        "- For each numbered action under a drone header (e.g., V_UAV_0), set `drone_id` to that header.\n"
        "- \"Take off from the current location.\" → takeoff {drone_id}\n"
        "- \"Land.\" → land {drone_id}\n"
        "- \"Go to the best approach point for [POI].\" → go_to_waypoint {drone_id, x, y, z}\n"
        "    Selection policy for (x,y): if POI has candidate_interaction_points, pick the FIRST candidate point; "
        "otherwise use the POI reference coordinates. For z use 2.0.\n"
        "- \"Inspect [POI]\" → perform_visual_inspection {drone_id, target_name:[POI]}\n"
        "- \"Return to the launch location.\" → return_to_launch {drone_id}\n"
        "- \"Wait [N] seconds\" → wait {drone_id, duration_seconds:N}\n"
        "Return only function calls.\n\n"
        "Plan:\n" + last_plan_text_;

    gemini_contents.push_back({
        {"role", "user"},
        {"parts", {{{"text", translator_user_text}}}}
    });
}

    json request_body;
    request_body["contents"] = gemini_contents;

    if (mode == RequestMode::TOOL_CALLS) {
        // 翻译阶段才带 tools
        request_body["toolConfig"] = {
            {"functionCallingConfig", {{"mode", "ANY"}}}
        };
        request_body["tools"] = { getGeminiToolDefinitions() };
    }

    // 2. 构建 systemInstruction
    std::string final_system_prompt;
    if (mode == RequestMode::REASONING_ONLY) {
        std::string dynamic_info_str = getSwarmStateAsText();
        final_system_prompt = system_prompt_content_ + "\n" + dynamic_info_str + R"(
Your output MUST be natural language reasoning ONLY.

Follow this template exactly:

---
**1. Reasoning:**
*   **Mission Goal:** [Briefly restate the user's high-level goal]
*   **Drone Assignments:**
    *   V_UAV_0 POIs: [List all POIs for V_UAV_0]
    *   V_UAV_1 POIs: [List all POIs for V_UAV_1]
*   **Pre-flight Check:**
    *   V_UAV_0 ...
    *   V_UAV_1 ...
*   **Full Action Sequence Plan (Text Version):**
    *   [Numbered list of actions for each drone]
---
**2. Summary:**
[Provide a brief, one-sentence summary for the user]

CRITICAL RULES:
- Do NOT include JSON.
- Do NOT include tool calls.
)";
} else { // TOOL_CALLS
    final_system_prompt = R"(
You are a translator. Convert the provided plan into tool calls using the given tool schema.
Do NOT rethink. Do NOT change order or content. Return only function calls (no natural language).
)";
}
    request_body["systemInstruction"] = {
        {"parts", {{{"text", final_system_prompt}}}}
    };

    request_body["generationConfig"] = {
        {"temperature", 0.0}
    };

    ROS_INFO_STREAM("--- Sending Gemini Request ---\n" << request_body.dump(2));

    // 3. 发起 HTTP 请求
    std::string model_name = "gemini-1.5-pro";
    std::string api_endpoint = "https://api.zhizengzeng.com/google/v1beta/models/" 
        + model_name + ":streamGenerateContent?key=" + api_key_;

    cpr::WriteCallback write_callback{[this](const std::string_view& data, intptr_t userdata) -> bool {
        if (interrupt_llm_request_.load()) return false;
        return this->streamCallbackForGemini(std::string(data));
    }};

    cpr::Response r = cpr::Post(
        cpr::Url{api_endpoint},
        cpr::Header{{"Content-Type", "application/json"}},
        cpr::Body{request_body.dump()},
        write_callback,
        cpr::Proxies{{"https", "http://127.0.0.1:7897"}}
    );

    // 4. 请求完成后处理
    if (interrupt_llm_request_.load()) return;

    if (r.status_code == 200) {
        if (current_mode == RequestMode::REASONING_ONLY) {
            // 缓存推理结果
            last_reasoning_text_ = accumulated_reasoning_response_;
            last_plan_text_      = extractPlanFromReasoning(last_reasoning_text_);
            has_cached_plan_.store(!last_plan_text_.empty());

            if (!accumulated_reasoning_response_.empty()) {
                publishReasoning(accumulated_reasoning_response_);
            }
            publishFeedback("Reasoning complete.");
            return; // 推理阶段不期待 tool calls
        }

        if (!accumulated_tool_calls_.empty()) {
            ROS_INFO_STREAM("Final collected tool calls: " << accumulated_tool_calls_.dump(2));
            publishFeedback("New plan received. Updating tasks for drones...");
            dispatchToolCalls(accumulated_tool_calls_, ReplanMode::REPLACE);
        } else {
            ROS_ERROR_STREAM("No tool calls parsed! Last raw reasoning: " 
                             << accumulated_reasoning_response_);
            publishFeedback("LLM finished without a clear action.");
        }
    } else {
        ROS_ERROR_STREAM("LLM API request failed with status " << r.status_code << ": " << r.text);
        publishFeedback("Error: LLM API request failed. Status: " + std::to_string(r.status_code));
    }
}


void LLMProcessorNode::processingLoop() {
ros::Rate rate(5); // 循环频率可以保持不变，每秒检查5次
while(ros::ok()) {
std::string command_to_process;
bool should_replan = false;

// 1. 检查是否有新的规划请求
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        if (needs_replan_) {
            should_replan = true;
            
            // 构建 "user" prompt 的逻辑保持不变
            command_to_process = "Re-evaluate the current situation.";
            if (!last_replan_reason_.empty()) {
                command_to_process += " Triggering event: " + last_replan_reason_;
            }
            if (!latest_user_command_.empty()) {
                command_to_process += "\nNew user instruction: '" + latest_user_command_ + "'";
            }
            
            // 重置标志
            needs_replan_ = false;
            latest_user_command_.clear();
            last_replan_reason_.clear();
        }
    }

    // 2. 如果需要规划，则启动一个后台线程
    if (should_replan) {
        // 使用互斥锁来保护对 llm_request_thread_ 的访问
        std::lock_guard<std::mutex> lock(llm_thread_mutex_);

        // 在启动新线程之前，确保上一个线程（如果存在）已经执行完毕并被 join
        // 这可以防止创建多个并行的 LLM 请求线程
        if (llm_request_thread_.joinable()) {
            llm_request_thread_.join();
        }
        
        // 启动一个新的线程来执行耗时的网络请求
        // this 指针和 command_to_process 作为参数传递给新线程
        ROS_INFO("Starting new LLM request in a background thread.");
        llm_request_thread_ = std::thread(&LLMProcessorNode::executeLlmRequest, this, command_to_process, RequestMode::REASONING_ONLY);
    }

    rate.sleep();
}

// 节点关闭前，最后一次检查并等待 LLM 线程结束
std::lock_guard<std::mutex> lock(llm_thread_mutex_);
if (llm_request_thread_.joinable()) {
    llm_request_thread_.join();
}
}

void LLMProcessorNode::trimConversationHistory() {
// 历史记录条目数 = 1 (system) + N * 2 (user/assistant turns)
// 我们保留 system prompt + MAX_CONVERSATION_TURNS 轮对话
const size_t max_messages = 1 + MAX_CONVERSATION_TURNS * 2;

if (conversation_history_.size() > max_messages) {
    // 从 system prompt 之后开始删除，直到数量达标
    conversation_history_.erase(
        conversation_history_.begin() + 1,
        conversation_history_.begin() + 1 + (conversation_history_.size() - max_messages)
    );
    ROS_INFO("Trimmed conversation history to the last %zu turns.", MAX_CONVERSATION_TURNS);
}
}

void LLMProcessorNode::dispatchToolCalls(const json& tool_calls, ReplanMode mode) {
// 按drone_id对tool_calls进行分组
std::map<std::string, std::vector<json>> tasks_by_drone;
for (const auto& tool_call : tool_calls) {
    try {
        const auto& arg_node = tool_call.at("function").at("arguments");
        json args = arg_node.is_string() ? json::parse(arg_node.get<std::string>())
                                         : arg_node;  // 兼容两种形态
        std::string drone_id = args.at("drone_id").get<std::string>();
        tasks_by_drone[drone_id].push_back(tool_call);
    } catch (const std::exception& e) {
        ROS_ERROR("Error parsing tool call for dispatch: %s", e.what());
    }
}


// 创建一个列表，用于存储需要被中断的无人机ID
// 我们将把阻塞的服务调用延迟到锁释放之后进行
std::vector<std::string> drones_to_interrupt;

// --- 步骤 1: 在持有锁的作用域内，只进行快速的内存操作 ---
{
    std::lock_guard<std::mutex> lock(contexts_map_mutex_);
    for (const auto& pair : tasks_by_drone) {
        const std::string& drone_id = pair.first;
        const auto& new_tasks = pair.second;

        if (drone_contexts_.count(drone_id)) {
            auto& context = drone_contexts_.at(drone_id);
            std::lock_guard<std::mutex> context_lock(context.context_data_mutex);

            if (mode == ReplanMode::REPLACE) {
                ROS_INFO("Replacing task queue for drone %s", drone_id.c_str());

                bool needs_interrupt = (context.state_enum == FLYING || context.state_enum == PERFORMING_ACTION);

                // 1. 清空旧队列
                context.tool_call_queue.clear();
                
                // 2. 如果需要中断，注入一个'wait'任务用于稳定过渡
                if (needs_interrupt) {
                    ROS_INFO("Injecting a 1.0s WAIT command to stabilize drone %s after interrupt.", drone_id.c_str());
                    
                    json wait_tool;
                    wait_tool["function"]["name"] = "wait";
                    json wait_args;
                    wait_args["drone_id"] = drone_id;
                    wait_args["duration_seconds"] = 1.0;
                    wait_tool["function"]["arguments"] = wait_args.dump();
                    context.tool_call_queue.push_front(wait_tool);

                    // CHANGED: 不在这里发送指令，而是将drone_id记录下来
                    drones_to_interrupt.push_back(drone_id);
                }
            }

            // 3. 将LLM生成的新任务追加到队列中
            for (const auto& task : new_tasks) {
                context.tool_call_queue.push_back(task);
            }
            ROS_INFO("Enqueued %zu new tasks for drone %s.", context.tool_call_queue.size(), drone_id.c_str());

            // REMOVED: 删除了原来在锁内的 'if (needs_interrupt)' 和 send_simple_command 调用
        }
    }
}
// --- 步骤 2: 在锁被释放后，安全地执行所有可能阻塞的网络/IO操作 ---
for (const auto& drone_id : drones_to_interrupt) {
    publishFeedback("Interrupting " + drone_id + " for new high-priority task.");
    // 这个ROS服务调用现在在锁外执行，不会导致死锁
    send_simple_command(drone_id, "wait", json{{"duration", 0.1}}.dump());
}
}

static std::string trim(const std::string& s) {
    auto a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    auto b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
}

std::string LLMProcessorNode::extractPlanFromReasoning(const std::string& md) {
    // 匹配你模板里的段落标题
    std::regex re(R"(Full Action Sequence Plan\s*\(Text Version\)\s*:\s*([\s\S]*?)(?:\n---|\n\*\*2\. Summary|\Z))");
    std::smatch m;
    if (std::regex_search(md, m, re) && m.size() > 1) {
        return trim(m[1].str());
    }
    // 兜底：找不到就全部返回（仍能被翻译器 prompt 正常处理）
    return trim(md);
}


json LLMProcessorNode::getGeminiToolDefinitions() {
    json declarations = json::array();

    // 1. Takeoff Tool
    declarations.push_back({
        {"name", "takeoff"},
        {"description", "Commands a specific drone to take off from the ground to a standard hover altitude."},
        {"parameters", {
            {"type", "object"},
            {"properties", {
                {"drone_id", {
                    {"type", "string"},
                    {"description", " The ID of the drone to perform this action. Must be a valid ID from the fleet list, e.g., 'V_UAV_0'."}
                }}
            }},
            {"required", json::array({"drone_id"})}
        }}
    });

    // 2. Land Tool
    declarations.push_back({
        {"name", "land"},
        {"description", "Commands a specific drone to land at its current position."},
        {"parameters", {
            {"type", "object"},
            {"properties", {
                {"drone_id", {
                    {"type", "string"},
                    {"description", "The drone ID to perform this action."}
                }}
            }},
            {"required", json::array({"drone_id"})}
        }}
    });

    // 3. Go To Waypoint Tool
    declarations.push_back({
        {"name", "go_to_waypoint"},
        {"description", "Commands a specific drone to fly to a specific 3D world coordinate."},
        {"parameters", {
            {"type", "object"},
            {"properties", {
                {"drone_id", {{"type", "string"}, {"description", "The drone ID to perform this action."}}},
                {"x", {{"type", "number"}, {"description", "Target X coordinate in meters."}}},
                {"y", {{"type", "number"}, {"description", "Target Y coordinate in meters."}}},
                {"z", {{"type", "number"}, {"description", "Target Z coordinate in meters."}}}
            }},
            {"required", json::array({"drone_id", "x", "y", "z"})}
        }}
    });

    // 4. Perform Visual Inspection Tool
    declarations.push_back({
        {"name", "perform_visual_inspection"},
        {"description", "PREFERRED METHOD: Commands a drone to perform a full, reliable visual inspection sequence at its current location, oriented towards a target POI. This is a compound macro-task."},
        {"parameters", {
            {"type", "object"},
            {"properties", {
                {"drone_id", {{"type", "string"}, {"description", "The drone ID to perform this action."}}},
                {"target_name", {
                    {"type", "string"},
                    {"description", "The exact name of the POI to inspect. You MUST use the key name 'target_name', e.g., 'House1'."}
                }}
            }},
            {"required", json::array({"drone_id", "target_name"})}
        }}
    });

    // 5. Wait Tool
    declarations.push_back({
        {"name", "wait"},
        {"description", "Commands a specific drone to hover at its current position for a specified number of seconds."},
        {"parameters", {
            {"type", "object"},
            {"properties", {
                {"drone_id", {{"type", "string"}, {"description", "The drone ID to perform this action."}}},
                {"duration_seconds", {
                    {"type", "number"},
                    {"description", "The duration to hover in seconds."}
                }}
            }},
            {"required", json::array({"drone_id", "duration_seconds"})}
        }}
    });

    // 6. Return to Launch Tool
    declarations.push_back({
        {"name", "return_to_launch"},
        {"description", "Commands a specific drone to return to its original launch point and hover, preparing for landing."},
        {"parameters", {
            {"type", "object"},
            {"properties", {
                {"drone_id", {
                    {"type", "string"},
                    {"description", "The drone ID to perform this action."}
                }}
            }},
            {"required", json::array({"drone_id"})}
        }}
    });

    // 7. Cancel All Tasks Tool
    declarations.push_back({
        {"name", "cancel_all_tasks"},
        {"description", "EMERGENCY: Immediately cancels the current and all queued tasks for a specific drone, causing it to hover."},
        {"parameters", {
            {"type", "object"},
            {"properties", {
                {"drone_id", {
                    {"type", "string"},
                    {"description", "The drone ID to cancel its action."}
                }}
            }},
            {"required", json::array({"drone_id"})}
        }}
    });

    // 8. Rotate Drone Yaw Relative Tool
    declarations.push_back({
        {"name", "rotate_drone_yaw_relative"},
        {"description", "EXPERT MODE: Commands a drone to rotate its yaw by a specified number of degrees relative to its current heading."},
        {"parameters", {
            {"type", "object"},
            {"properties", {
                {"drone_id", {{"type", "string"}, {"description", "The drone ID to perform this action."}}},
                {"angle_degrees", {
                    {"type", "number"},
                    {"description", "The relative angle to rotate in degrees. Positive is clockwise, negative is counter-clockwise."}
                }}
            }},
            {"required", json::array({"drone_id", "angle_degrees"})}
        }}
    });

    // 9. Backup Tool
    declarations.push_back({
        {"name", "backup"},
        {"description", "EXPERT MODE: Commands a drone to move backward from its current position by a specified distance."},
        {"parameters", {
            {"type", "object"},
            {"properties", {
                {"drone_id", {{"type", "string"}, {"description", "The drone ID to perform this action."}}},
                {"distance", {
                    {"type", "number"},
                    {"description", "The distance moved backwards (in meters)."}
                }}
            }},
            {"required", json::array({"drone_id", "distance"})}
        }}
    });

    // 10. Adjust Camera Pitch Tool
    declarations.push_back({
        {"name", "adjust_camera_pitch"},
        {"description", "EXPERT MODE: Adjusts the drone's camera pitch angle."},
        {"parameters", {
            {"type", "object"},
            {"properties", {
                {"drone_id", {{"type", "string"}, {"description", "The drone ID to perform this action."}}},
                {"pitch_angle", {
                    {"type", "number"},
                    {"description", "Target pitch angle in degrees. Negative values tilt up, positive values tilt down."}
                }}
            }},
            {"required", json::array({"drone_id", "pitch_angle"})}
        }}
    });

    return {{"function_declarations", declarations}};
}
std::string LLMProcessorNode::send_simple_command(const std::string& drone_id, const std::string& action_type, const std::string& params_json) {
nlp_drone_control::ExecuteDroneAction srv;
srv.request.action_type = action_type;
srv.request.params_json = params_json;

if (execute_action_clients_.at(drone_id).call(srv) && srv.response.success) {
    return action_type + " command sent successfully to " + drone_id;
}

std::string error_msg = "Error: Failed to send " + action_type + " command to " + drone_id;
if (!srv.response.message.empty()) {
    error_msg += ". Reason: " + srv.response.message;
}
ROS_ERROR_STREAM(error_msg);
return error_msg;
}

std::string LLMProcessorNode::cancel_all_tasks_action(const std::string& drone_id) {
ROS_WARN("Executing cancel_all_tasks for drone %s", drone_id.c_str());
{
std::lock_guard<std::mutex> lock(contexts_map_mutex_);
auto& context = drone_contexts_.at(drone_id);
// NEW: Also abort any running macro task
context.is_in_macro_task = false;
std::lock_guard<std::mutex> context_lock(context.context_data_mutex);
context.tool_call_queue.clear();
}
// CHANGED: Use the correct function
send_simple_command(drone_id, "wait", json{{"duration_seconds", 1.0}}.dump());
return "All tasks for " + drone_id + " cancelled. Drone is returning to hover state.";
}
std::string LLMProcessorNode::return_to_launch_action(const std::string& drone_id) {
geometry_msgs::Point launch_pos;
bool has_launch_pos = false;
{
std::lock_guard<std::mutex> lock(contexts_map_mutex_);
auto& context = drone_contexts_.at(drone_id);
if (context.state_enum != HOVERING) return "Error: Drone must be hovering to return to launch.";
if (context.has_recorded_launch_position) {
launch_pos = context.launch_position;
has_launch_pos = true;
} else {
return "Error: Launch position for " + drone_id + " is unknown.";
}
}

if (has_launch_pos) {
    //ROS_INFO_STREAM("Commanding " << drone_id << " to return to launch point (" 
                    //<< launch_pos.x << ", " << launch_pos.y << ", " << launch_pos.z << ")");
    return go_to_waypoint_action(launch_pos.x, launch_pos.y, 2.0, drone_id);
}
return "Error: Should not happen, failed to retrieve launch position.";
}

void LLMProcessorNode::executionLoop(const std::string& drone_id) {
ros::Rate rate(5);
while (ros::ok()) {
// --- 1. 检查导航是否超时 ---
{
std::lock_guard<std::mutex> lock(contexts_map_mutex_);
auto& context = drone_contexts_.at(drone_id);
const ros::Duration NAVIGATION_TIMEOUT(60.0);
if (context.state_enum == FLYING && !context.navigation_start_time_.is_zero() &&
(ros::Time::now() - context.navigation_start_time_ > NAVIGATION_TIMEOUT))
{
ROS_ERROR("Drone %s navigation timed out!", drone_id.c_str());
publishFeedback("Error: Plan for " + drone_id + " aborted due to navigation timeout.");

std::lock_guard<std::mutex> context_lock(context.context_data_mutex);
            context.tool_call_queue.clear();
            context.is_task_executing = false;
            
            // 发送悬停指令作为安全措施
            send_simple_command(drone_id, "hover");
            context.navigation_start_time_ = ros::Time(0); // 重置计时器
        }
    }
      
    bool can_start_new_task = false;
    {
        std::lock_guard<std::mutex> lock(contexts_map_mutex_);
        auto& context = drone_contexts_.at(drone_id);
        // 这是我们反复强调的核心状态机检查
        if ((context.state_enum == HOVERING || context.state_enum == GROUNDED) && !context.is_task_executing) {
            if (!context.tool_call_queue.empty()) {
                can_start_new_task = true;
            }
        }
    }

    if (can_start_new_task) {
        json next_tool;
        {
            std::lock_guard<std::mutex> lock(contexts_map_mutex_);
            auto& context = drone_contexts_.at(drone_id);
            std::lock_guard<std::mutex> context_lock(context.context_data_mutex);
            next_tool = context.tool_call_queue.front();
            context.tool_call_queue.pop_front();
            context.is_task_executing = true; // **标记任务开始执行**
        }
        
        // executeTool现在是非阻塞的，它会立即返回
        std::string result = executeTool(next_tool, drone_id);
        
        // 只有在指令发送失败时，才立即重置状态并清空队列。
        // 如果发送成功，`is_task_executing` 必须保持为 `true`，直到回调函数确认任务完成。
        if (result.rfind("Error:", 0) == 0) {
           std::lock_guard<std::mutex> lock(contexts_map_mutex_);
           auto& context = drone_contexts_.at(drone_id);
           std::lock_guard<std::mutex> context_lock(context.context_data_mutex);
           context.is_task_executing = false; // 发送失败，所以任务并未开始
           context.tool_call_queue.clear();   // 清空后续所有计划，因为出错了
           publishFeedback("Plan for " + drone_id + " aborted due to command error: " + result);
        }
    }
    rate.sleep();
}
}

void LLMProcessorNode::publishGoal(const geometry_msgs::PoseStamped& goal_pose, const std::string& drone_id) {
if (llm_goal_pubs_.count(drone_id)) {
llm_goal_pubs_.at(drone_id).publish(goal_pose);
}
}

std::string LLMProcessorNode::go_to_waypoint_action(double x, double y, double z, const std::string& drone_id) {
{
std::lock_guard<std::mutex> lock(contexts_map_mutex_);
auto& context = drone_contexts_.at(drone_id);
if (context.state_enum != HOVERING) return "Error: Drone must be hovering.";
//context.state_enum = FLYING;
context.navigation_start_time_ = ros::Time::now(); // 记录开始时间
}

geometry_msgs::PoseStamped goal_pose;
goal_pose.header.stamp = ros::Time::now();
goal_pose.header.frame_id = "map";
goal_pose.pose.position.x = x;
goal_pose.pose.position.y = y;
goal_pose.pose.position.z = z;
goal_pose.pose.orientation.w = 1.0;

publishGoal(goal_pose, drone_id);
return "Goal sent to A* planner for " + drone_id;
}

std::string LLMProcessorNode::executeTool(const json& tool_call, const std::string& drone_id){
std::string tool_name;
json tool_input;
try {
    tool_name = tool_call.at("function").at("name").get<std::string>();

    const auto& arg_node = tool_call.at("function").at("arguments");
    tool_input = arg_node.is_string() ? json::parse(arg_node.get<std::string>())
                                      : arg_node; // 兼容两种形态
} catch (const json::exception& e) {
    std::string error_msg = "Error: Failed to parse tool call JSON: " + std::string(e.what());
    ROS_ERROR_STREAM(error_msg);
    publishFeedback(error_msg);
    return error_msg;
}

ROS_INFO_STREAM("[" << drone_id << "] Executing tool: " << tool_name << " with input: " << tool_input.dump());
std::string result_msg = "";
if (tool_name == "takeoff") {
        {
        std::lock_guard<std::mutex> lock(contexts_map_mutex_);
        auto& context = drone_contexts_.at(drone_id);
        // 假设起飞时无人机的位置就是其当前位置
        context.launch_position = context.pose.pose.position;
        context.has_recorded_launch_position = true;
        ROS_INFO_STREAM("Recorded launch position for " << drone_id << " at (x:" 
            << context.launch_position.x << ", y:" << context.launch_position.y 
            << ", z:" << context.launch_position.z << ")");
    }
    result_msg = send_simple_command(drone_id, "takeoff", json{{"altitude", 3.0}}.dump());
} else if (tool_name == "land") {
    result_msg = send_simple_command(drone_id, "land");
} else if (tool_name == "go_to_waypoint") {
    if (tool_input.contains("x") && tool_input.contains("y") && tool_input.contains("z")) {
        result_msg = go_to_waypoint_action(
            tool_input["x"].get<double>(),
            tool_input["y"].get<double>(),
            tool_input["z"].get<double>(),
            drone_id
        );
    } else {
        result_msg = "Error: Missing x, y, or z parameters for go_to_waypoint.";
    }
} else if (tool_name == "return_to_launch") {
    result_msg = return_to_launch_action(drone_id);
} else if (tool_name == "wait") {
    if (tool_input.contains("duration_seconds")) {
        result_msg = send_simple_command(drone_id, "wait", json{{"duration", tool_input["duration_seconds"].get<double>()}}.dump());
    } else {
        result_msg = "Error: 'duration_seconds' is missing for wait tool.";
    }
} else if (tool_name == "cancel_all_tasks") {
    result_msg = cancel_all_tasks_action(drone_id);
} 
// IMPORTANT: a complex blocking task has been removed.
// You need to handle it by adding smaller, non-blocking tools.
else if (tool_name == "perform_visual_inspection") {
if (tool_input.contains("target_name")) {
start_visual_inspection_macro(tool_input["target_name"].get<std::string>(), drone_id);
result_msg = "Starting visual inspection macro-task for " + drone_id;
} else {
result_msg = "Error: 'target_name' is missing for perform_visual_inspection.";
}
}
else if (tool_name == "rotate_drone_yaw_relative") {
result_msg = send_simple_command(drone_id, "rotate_drone_yaw_relative", tool_input.dump());
}
else if (tool_name == "backup") {
result_msg = send_simple_command(drone_id, "backup", tool_input.dump());
}
else if (tool_name == "adjust_camera_pitch") {
result_msg = send_simple_command(drone_id, "adjust_camera_pitch", tool_input.dump());
}
else {
result_msg = "Error: Unknown or unsupported tool: " + tool_name;
}

if (result_msg.rfind("Error:", 0) != 0) {
    publishFeedback("LLM Action [" + drone_id + "]: " + result_msg);
}

return result_msg;
}

void LLMProcessorNode::handle_task_completion(const std::string& drone_id) {
NextMacroAction next_action; // 用于存储下一步宏动作

// --- 步骤 1: 在锁内，更新状态并决定下一步动作 ---
{
    std::lock_guard<std::mutex> lock(contexts_map_mutex_);
    if (!drone_contexts_.count(drone_id)) return;
    
    auto& context = drone_contexts_.at(drone_id);
    
    if (context.is_in_macro_task) {
        ROS_INFO("[%s] Macro sub-step complete. Advancing macro...", drone_id.c_str());
        next_action = advance_visual_inspection_macro(drone_id);
        // 注意：我们不再需要 is_macro_now_finished 标志了
    } else {
        // 这是一个简单的单步任务完成了
        std::lock_guard<std::mutex> context_lock(context.context_data_mutex);
        context.is_task_executing = false;
        ROS_INFO("[%s] Simple task complete.", drone_id.c_str());
    }
} // --- 锁在这里被释放 ---

// --- 步骤 2: 在锁外，执行已决定的动作 ---
if (next_action.is_valid) {
    if (next_action.is_capture_action) {
        // 执行拍照动作
        sensor_msgs::Image captured_image;
        std::string save_dir;
        {
            std::lock_guard<std::mutex> lock(contexts_map_mutex_);
            save_dir = drone_contexts_.at(drone_id).macro_task_data.value("save_directory", "/tmp");
        }

        if (capture_and_get_image(save_dir, next_action.capture_filename, captured_image, drone_id)) {
            {
                std::lock_guard<std::mutex> lock(contexts_map_mutex_);
                drone_contexts_.at(drone_id).collected_images_for_vlm.push_back(captured_image);
            }
            // 拍照成功后，递归调用以推进宏
            handle_task_completion(drone_id);
        } else {
            publishFeedback("Error: [" + drone_id + "] Failed to capture image. Aborting inspection.");
            std::lock_guard<std::mutex> lock(contexts_map_mutex_);
            auto& context = drone_contexts_.at(drone_id);
            context.is_in_macro_task = false;
            context.is_task_executing = false;
        }
    } else if (next_action.action_type == "ANALYZE_VLM") {
        // 执行VLM分析
        vlm_service::DescribeScene srv;
        {
            std::lock_guard<std::mutex> lock(contexts_map_mutex_);
            srv.request.images = drone_contexts_.at(drone_id).collected_images_for_vlm;
        }
        srv.request.prompt = "你收到了从无人机同一位置、多角度拍摄的五张照片(上、下、前、左、右)。请综合分析所有信息，提供一个关于当前场景的全面描述，重点关注设备状态或潜在异常。";
        
        if (describe_scene_client_.call(srv) && srv.response.success) {
            publishFeedback("Visual Inspection Report: " + srv.response.description);
        } else {
            publishFeedback("Error: [" + drone_id + "] Failed to get a description from the VLM service.");
        }
        // 分析完成后，递归调用以正式结束宏
        handle_task_completion(drone_id);

    } else {
        // 执行普通的简单命令
        send_simple_command(drone_id, next_action.action_type, next_action.params_json);
    }
}
// 注意：步骤3（触发重新规划的逻辑）已被完全移除。
}

double LLMProcessorNode::calculate_yaw_to_target(const std::string& target_name, const geometry_msgs::PoseStamped& drone_pose)
{
//geometry_msgs::PoseStamped stable_pose;
geometry_msgs::Point target_pos = _pois.at(target_name).position;
//{
//    std::lock_guardstd::mutex lock(contexts_map_mutex);
//   drone_pose = drone_contexts_.at(drone_id).pose;
//}

tf2::Quaternion q_ros;
tf2::fromMsg(drone_pose.pose.orientation, q_ros);
tf2::Vector3 forward_vec_ros_3d = tf2::Transform(q_ros) * tf2::Vector3(1, 0, 0);
tf2::Vector3 target_dir_ros_3d(target_pos.x - drone_pose.pose.position.x,
                               target_pos.y - drone_pose.pose.position.y,
                               target_pos.z - drone_pose.pose.position.z);
tf2::Vector3 forward_vec_ros_2d = forward_vec_ros_3d;
forward_vec_ros_2d.setZ(0);
if (forward_vec_ros_2d.length() < 0.001) {
    ROS_WARN("Drone is pointing vertically. Using a default forward direction for yaw calculation.");
    forward_vec_ros_2d.setValue(1.0, 0.0, 0.0);
}
forward_vec_ros_2d.normalize();
tf2::Vector3 target_dir_ros_2d = target_dir_ros_3d;
target_dir_ros_2d.setZ(0);
if (target_dir_ros_2d.length() < 0.001) {
    ROS_ERROR("Target is directly above or below the drone. Cannot determine a unique yaw direction.");
    return DBL_MAX; 
}
target_dir_ros_2d.normalize();
double current_yaw_rad_ros = atan2(forward_vec_ros_2d.y(), forward_vec_ros_2d.x());
double target_yaw_rad_ros  = atan2(target_dir_ros_2d.y(), target_dir_ros_2d.x());
double delta_yaw_rad_ros = target_yaw_rad_ros - current_yaw_rad_ros;
while (delta_yaw_rad_ros > M_PI)  delta_yaw_rad_ros -= 2.0 * M_PI;
while (delta_yaw_rad_ros <= -M_PI) delta_yaw_rad_ros += 2.0 * M_PI;
return - (delta_yaw_rad_ros * 180.0 / M_PI);
}

void LLMProcessorNode::start_visual_inspection_macro(const std::string& target_name, const std::string& drone_id) {
NextMacroAction first_action;

// --- 在锁内：初始化状态 ---
{
    std::lock_guard<std::mutex> lock(contexts_map_mutex_);
    
    if (!drone_contexts_.count(drone_id)) return;
    auto& context = drone_contexts_.at(drone_id);

    if (context.state_enum != HOVERING) {
        ROS_ERROR("[%s] Macro Error: Cannot start inspection, drone is not hovering.", drone_id.c_str());
        std::lock_guard<std::mutex> context_lock(context.context_data_mutex);
        context.is_task_executing = false;
        return;
    }
    if (_pois.find(target_name) == _pois.end()) {
        ROS_ERROR("[%s] Macro Error: POI '%s' not found.", drone_id.c_str(), target_name.c_str());
        std::lock_guard<std::mutex> context_lock(context.context_data_mutex);
        context.is_task_executing = false;
        return;
    }

    ROS_INFO("[%s] Initializing Visual Inspection Macro for target '%s'", drone_id.c_str(), target_name.c_str());
    
    std::lock_guard<std::mutex> context_lock(context.context_data_mutex);
    context.is_in_macro_task = true;
    context.current_macro_name = "visual_inspection";
    context.macro_task_step = 0; // 重置为0，因为 advance_macro 会先++
    
    context.macro_task_data.clear();
    context.macro_task_data["target_name"] = target_name;
    context.collected_images_for_vlm.clear();
    
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
    std::string save_directory = image_save_base_path_ + "/" + drone_id + "/" + ss.str();
    
    try {
        std::filesystem::create_directories(save_directory);
        context.macro_task_data["save_directory"] = save_directory;
        publishFeedback("Photos for [" + drone_id + "] will be saved to: " + save_directory);
    } catch (const std::filesystem::filesystem_error& e) {
        ROS_ERROR_STREAM("Macro Error: Failed to create save directory: " << e.what());
        context.is_in_macro_task = false;
        context.is_task_executing = false;
        publishFeedback("Error: [" + drone_id + "] Aborting inspection due to file system error.");
        return;
    }
    
    // 决定第一步动作
    first_action = advance_visual_inspection_macro(drone_id);
} // --- 锁在这里被释放 ---

// --- 在锁外：执行第一步动作 ---
if (first_action.is_valid) {
    // 由于这是一个全新的宏任务，我们不需要担心拍照等复杂情况
    // 直接发送简单命令即可
    send_simple_command(drone_id, first_action.action_type, first_action.params_json);
}
}

NextMacroAction LLMProcessorNode::advance_visual_inspection_macro(const std::string& drone_id) {
// 这个函数现在在锁内被调用，但它本身不执行任何阻塞操作

if (!drone_contexts_.count(drone_id)) return {};
auto& context = drone_contexts_.at(drone_id);
if (!context.is_in_macro_task) {
    ROS_WARN("[%s] advance_macro called, but drone is no longer in a macro. Aborting.", drone_id.c_str());
    return {};
}

context.macro_task_step++;

std::string target_name = context.macro_task_data.value("target_name", "UNKNOWN");
const double INSPECTION_YAW_ANGLE = 60.0;

NextMacroAction next_action;
next_action.is_valid = true; // 默认是有动作的

switch (context.macro_task_step) {
    // === 阶段 1: 准备动作 ===
    case 1:
        publishFeedback("[" + drone_id + "] Inspection Step 1/13: Stabilizing (2s)...");
        next_action.action_type = "wait";
        next_action.params_json = json{{"duration", 2.0}}.dump();
        break;
    case 2:
        publishFeedback("[" + drone_id + "] Inspection Step 2/13: Orienting towards target...");
        {
            double angle_to_rotate = calculate_yaw_to_target(target_name, context.pose);
            if (angle_to_rotate == DBL_MAX) {
                publishFeedback("Error: [" + drone_id + "] Could not calculate orientation. Aborting inspection.");
                context.is_in_macro_task = false;
                context.is_task_executing = false;
                next_action.is_valid = false;
            } else if (std::abs(angle_to_rotate) > 5.0) {
                next_action.action_type = "rotate_drone_yaw_relative";
                next_action.params_json = json{{"angle_degrees", angle_to_rotate}}.dump();
            } else {
                publishFeedback("[" + drone_id + "] Already facing target, skipping rotation.");
                // 跳过这一步，递归调用自己来进入下一步
                return advance_visual_inspection_macro(drone_id); 
            }
        }
        break;
    case 3:
        publishFeedback("[" + drone_id + "] Inspection Step 3/13: Backing up (2m)...");
        next_action.action_type = "backup";
        next_action.params_json = json{{"distance", 2.0}}.dump();
        break;

    // === 阶段 2: 垂直扫描拍照 ===
    case 4:
        publishFeedback("[" + drone_id + "] Inspection Step 4/13: Tilting camera up (-45 deg)...");
        next_action.action_type = "adjust_camera_pitch";
        next_action.params_json = json{{"pitch_angle", -45.0}}.dump();
        break;
    case 5:
        publishFeedback("[" + drone_id + "] Inspection Step 5/13: Capturing 'up' image...");
        next_action.is_capture_action = true;
        next_action.capture_filename = "0_up.jpg";
        break;
    case 6:
        publishFeedback("[" + drone_id + "] Inspection Step 6/13: Tilting camera down (+45 deg)...");
        next_action.action_type = "adjust_camera_pitch";
        next_action.params_json = json{{"pitch_angle", 45.0}}.dump();
        break;
    case 7:
        publishFeedback("[" + drone_id + "] Inspection Step 7/13: Capturing 'down' image...");
        next_action.is_capture_action = true;
        next_action.capture_filename = "1_down.jpg";
        break;
    case 8:
        publishFeedback("[" + drone_id + "] Inspection Step 8/13: Leveling camera (0 deg)...");
        next_action.action_type = "adjust_camera_pitch";
        next_action.params_json = json{{"pitch_angle", 0.0}}.dump();
        break;
    case 9:
        publishFeedback("[" + drone_id + "] Inspection Step 9/13: Capturing 'forward' image...");
        next_action.is_capture_action = true;
        next_action.capture_filename = "2_forward.jpg";
        break;

    // === 阶段 3: 水平扫描拍照 ===
    case 10:
        publishFeedback("[" + drone_id + "] Inspection Step 10/13: Rotating left (-60 deg)...");
        next_action.action_type = "rotate_drone_yaw_relative";
        next_action.params_json = json{{"angle_degrees", -INSPECTION_YAW_ANGLE}}.dump();
        break;
    case 11:
        publishFeedback("[" + drone_id + "] Inspection Step 11/13: Capturing 'left' image...");
        next_action.is_capture_action = true;
        next_action.capture_filename = "3_left.jpg";
        break;
    case 12:
        publishFeedback("[" + drone_id + "] Inspection Step 12/13: Rotating right (+120 deg)...");
        next_action.action_type = "rotate_drone_yaw_relative";
        next_action.params_json = json{{"angle_degrees", 2.0 * INSPECTION_YAW_ANGLE}}.dump();
        break;
    case 13:
        publishFeedback("[" + drone_id + "] Inspection Step 13/13: Capturing 'right' image...");
        next_action.is_capture_action = true;
        next_action.capture_filename = "4_right.jpg";
        break;

    // === 阶段 4: 恢复与分析 ===
    case 14:
        publishFeedback("[" + drone_id + "] Inspection Step 14/15: Returning to center yaw...");
        next_action.action_type = "rotate_drone_yaw_relative";
        next_action.params_json = json{{"angle_degrees", -INSPECTION_YAW_ANGLE}}.dump();
        break;
    case 15:
         publishFeedback("[" + drone_id + "] Inspection Step 15/15: All images captured. Requesting VLM analysis...");
        // VLM 分析是宏任务的最后一步，它也是阻塞的，但我们将在锁外调用它。
        // 我们用一个特殊的 action_type 来标记它。
        next_action.action_type = "ANALYZE_VLM";
        break;

    // === 阶段 5: 宏任务结束 ===
    default:
        publishFeedback("[" + drone_id + "] Visual Inspection Macro COMPLETED.");
        {
            std::lock_guard<std::mutex> context_lock(context.context_data_mutex);
            context.is_in_macro_task = false;
            context.is_task_executing = false; // 标记整个宏任务工具已完成
            context.collected_images_for_vlm.clear();
            context.macro_task_data.clear();
        }
        next_action.is_valid = false; // 标记宏结束，没有后续动作
        break;
}
return next_action;
}

void LLMProcessorNode::publishFeedback(const std::string& text) {
std_msgs::String msg;
msg.data = text;
central_nlp_feedback_pub_.publish(msg);
}
void LLMProcessorNode::publishReasoning(const std::string& text) {
std_msgs::String msg;
msg.data = text;
reasoning_pub_.publish(msg); // ++ 这是正确的，它发布到了 reasoning topic
}
bool LLMProcessorNode::capture_and_get_image(const std::string& dir, const std::string& filename, sensor_msgs::Image& out_image, const std::string& drone_id) {

std::string image_topic_to_capture = "/" + drone_id + "/camera/image_raw";

vlm_service::CaptureImage srv;
srv.request.directory_path = dir;
srv.request.file_name = filename;
srv.request.image_topic = image_topic_to_capture; 

ROS_INFO_STREAM("Requesting capture for " << drone_id << " from topic " << image_topic_to_capture);

if (capture_image_client_.call(srv) && srv.response.success) {
    ROS_INFO_STREAM("Successfully captured image for '" << drone_id << "' via service. Saved to " << srv.response.file_path);
    out_image = srv.response.captured_image;
    ros::Duration(0.2).sleep();
    return true;
} else {
    ROS_ERROR_STREAM("Failed to capture image for " << drone_id << ". Service message: " << srv.response.message);
    return false;
}
}

tf2::Vector3 LLMProcessorNode::transformRosToUnity(const tf2::Vector3& ros_vec) {
return tf2::Vector3(
-ros_vec.y(), // Unity X is ROS -Y
ros_vec.z(), // Unity Y is ROS +Z
ros_vec.x()  // Unity Z is ROS +X
);
}

int main(int argc, char** argv) {
ros::init(argc, argv, "llm_processor_node");
ros::NodeHandle nh;
ros::MultiThreadedSpinner spinner; // 使用多线程Spinner，它会自动管理线程
LLMProcessorNode node(nh);
spinner.spin();
return 0;
}
