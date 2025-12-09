#include "nlp_drone_control/llm_processor_node.h" 

LLMProcessorNode::LLMProcessorNode(ros::NodeHandle& nh):nh_(nh),_drone_state(GROUNDED), _drone_status("IDLE"),_is_task_executing(false),_action_completed(false),
_has_recorded_launch_position(false){

    ROS_INFO("LLM Processor Node (C++) is running.");

    // =================================================================================
    // === 1. Load Configuration to match your specific launch file structure        ===
    // =================================================================================
    
    // --- Critical Parameters (from <param> tags, using getParam) ---
    // These names match your launch file exactly.
    if (!nh_.getParam("deepseek_api_key", api_key_) || api_key_.empty()) {
        ROS_WARN("API Key not set! Please set the 'deepseek_api_key' parameter in your launch file.");
    }

    std::string prompt_file_path;
    if (!nh_.getParam("system_prompt_file", prompt_file_path)) {
        ROS_ERROR("Parameter 'system_prompt_file' not found. Using fallback content.");
        system_prompt_content_ = "You are an expert drone control assistant."; // Fallback
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

    // --- ROS Interface Parameters (from <param> tags, using param() for robustness) ---
    std::string nlp_feedback_topic, nlp_command_topic, drone_pose_topic, drone_status_topic;
    std::string llm_goal_topic, drone_target_topic, action_feedback_topic;
    std::string describe_scene_service, execute_action_service, capture_image_service;
    
    // The first argument to nh_.param matches your <param name="..."> exactly.
    // The third argument is a default value, in case the <param> tag is ever removed from the launch file.
    nh_.param<std::string>("nlp_feedback_topic", nlp_feedback_topic, "/nlp_feedback");
    nh_.param<std::string>("nlp_command_topic", nlp_command_topic, "/nlp_command");
    nh_.param<std::string>("drone_pose_topic", drone_pose_topic, "/drone_pose");
    nh_.param<std::string>("drone_status_topic", drone_status_topic, "/drone_status");
    nh_.param<std::string>("llm_goal_topic", llm_goal_topic, "/llm_goal");
    nh_.param<std::string>("drone_target_topic", drone_target_topic, "/drone_target");
    nh_.param<std::string>("action_feedback_topic", action_feedback_topic, "/drone_action_feedback");

    nh_.param<std::string>("describe_scene_service", describe_scene_service, "/vision_service/describe_scene");
    nh_.param<std::string>("execute_action_service", execute_action_service, "/execute_drone_action");
    nh_.param<std::string>("capture_image_service", capture_image_service, "/vision_service/capture_image");
    
    _current_pose.pose.position.x = 0.0;
    _current_pose.pose.position.y = 0.0;
    _current_pose.pose.position.z = 0.0; 
    _current_pose.pose.orientation.x = 0.0;
    _current_pose.pose.orientation.y = 0.0;
    _current_pose.pose.orientation.z = 0.0;
    _current_pose.pose.orientation.w = 1.0; 
    
    // Initialize Publishers and Subscribers using the loaded variables
    ROS_INFO("--- Initializing ROS Interfaces from launch file parameters ---");
    nlp_feedback_pub_ = nh_.advertise<std_msgs::String>(nlp_feedback_topic, 10);
    nlp_command_sub_ = nh_.subscribe(nlp_command_topic, 1, &LLMProcessorNode::nlpCommandCallback, this);
    _current_pose_sub_ = nh_.subscribe(drone_pose_topic, 1, &LLMProcessorNode::currentPoseCallback, this);
    _drone_status_sub_ = nh_.subscribe(drone_status_topic, 1, &LLMProcessorNode::droneStatusCallback, this);
    llm_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(llm_goal_topic, 1);
    direct_unity_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(drone_target_topic, 1); 
    _action_feedback_sub = nh_.subscribe(action_feedback_topic, 1, &LLMProcessorNode::actionFeedbackCallback, this);

XmlRpc::XmlRpcValue poi_list_param;
if (nh_.getParam("points_of_interest", poi_list_param)) {
    ROS_INFO("Loading Points of Interest from YAML (Hybrid Strategy)...");

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
                ROS_INFO_STREAM("Loaded POI: " << current_poi_data.name);

            } else {
                ROS_WARN_STREAM("Entry at index " << i << " in 'points_of_interest' is not a dictionary/struct. Skipping.");
            }
        }
    } else {
        ROS_WARN("Parameter 'points_of_interest' is not a list/array type. Check your YAML.");
    }
} else {
    ROS_WARN("No 'points_of_interest' found in ROS parameters. Make sure your YAML is loaded.");
}
if (_pois.count("V_UAV_0")) {
    { 
        std::lock_guard<std::mutex> lock_pose(_pose_mutex); 
        _current_pose.pose.position = _pois["V_UAV_0"].position;
    }
} else {
    ROS_WARN("V_UAV_0 is not found!");
}
    conversation_history_ = json::array();
    conversation_history_.push_back({
        {"role", "system"},
        // 这里的 content 之后会在processingLoop中动态构建
        {"content", ""} 
    });

    // --- Initialize Service Clients using loaded variables ---
    ROS_INFO("--- Waiting for required services ---");
    describe_scene_client_ = nh_.serviceClient<vlm_service::DescribeScene>(describe_scene_service);
    execute_action_client_ = nh_.serviceClient<nlp_drone_control::ExecuteDroneAction>(execute_action_service);
    capture_image_client_ = nh_.serviceClient<vlm_service::CaptureImage>(capture_image_service);
    
    describe_scene_client_.waitForExistence();
    execute_action_client_.waitForExistence();
    capture_image_client_.waitForExistence();
    
    ROS_INFO("All services are available. Node is fully initialized.");

    _processing_thread = std::thread(&LLMProcessorNode::processingLoop, this);
}
LLMProcessorNode::~LLMProcessorNode() {
    if (_processing_thread.joinable()) {
        _processing_thread.join(); // 等待线程结束
    }
}
void LLMProcessorNode::actionFeedbackCallback(const std_msgs::String::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(_feedback_mutex);
    if (msg->data == "ACTION_COMPLETE") {
        ROS_INFO("Received 'ACTION_COMPLETE' feedback from Unity.");
        _action_completed = true;
    }
}
void LLMProcessorNode::droneStatusCallback(const std_msgs::String::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(_status_mutex); 
    _drone_status = msg->data;
    
    if (_drone_status == "IDLE_ON_GROUND") {
        _drone_state = GROUNDED;
    } else if (_drone_status == "NAVIGATING") {
        _drone_state = FLYING;
    } else if (_drone_status == "HOVERING" || _drone_status == "IDLE" || _drone_status == "IDLE_IN_AIR") {
        _drone_state = HOVERING;
    } else if (_drone_status == "PERFORMING_ACTION") {
        _drone_state = PERFORMING_ACTION;
    } else if (_drone_status == "TAKING_OFF") { 
        _drone_state = TAKING_OFF;
    } else if (_drone_status == "LANDING") {   
        _drone_state = LANDING;
    } else {
        ROS_WARN_STREAM("Received unknown drone status from Unity: " << _drone_status);
    }  
}

bool LLMProcessorNode::streamCallback(const std::string& chunk) {
    std::stringstream ss(chunk);
    std::string line;

    while (std::getline(ss, line)) {
        if (line.rfind("data: ", 0) == 0) {
            std::string data_str = line.substr(6);

            if (data_str == "[DONE]") {
                // 在流结束时，如果累积了思维链内容，完整地发布一次
                //if (!accumulated_reasoning_response_.empty()) {
                //    publishFeedback("LLM Reasoning:\n---\n" + accumulated_reasoning_response_ + "\n---");
                //}
                                // 流结束时，确保缓冲区里最后一部分内容也被发布出去
                if (!accumulated_reasoning_response_.empty()) {
                    publishFeedback("LLM Reasoning: " + accumulated_reasoning_response_);
                    accumulated_reasoning_response_.clear(); // 清空缓冲区
                }
                return true;
            }

            try {
                json delta_json = json::parse(data_str);
                if (delta_json["choices"][0].contains("delta")) {
                    const auto& delta = delta_json["choices"][0]["delta"];

                  //处理思维链
                    if (delta.contains("reasoning_content") && delta["reasoning_content"].is_string()) {
                        std::string reasoning_part = delta["reasoning_content"];
                        accumulated_reasoning_response_ += reasoning_part;
  // 2. 检查缓冲区中是否包含换行符，如果包含，就发布换行符之前的所有内容
                        size_t newline_pos;
                        while ((newline_pos = accumulated_reasoning_response_.find('\n')) != std::string::npos) {
                            // 提取一行完整的内容
                            std::string line_to_publish = accumulated_reasoning_response_.substr(0, newline_pos);
                            
                            // 发布这一行 (可以加上前缀)
                            if (!line_to_publish.empty()) {
                                publishFeedback("LLM Reasoning: " + line_to_publish);
                            }
                            
                            // 从缓冲区中移除已经发布的内容（包括换行符）
                            accumulated_reasoning_response_.erase(0, newline_pos + 1);
                        }
                        // 为了实时感，可以逐块发布，但可能会刷屏。或者像上面一样，结束后发一次。
                        //publishFeedback("LLM is thinking: " + reasoning_part); 
                    }

                    if (delta.contains("content") && delta["content"].is_string()) {
                        std::string text_part = delta["content"];
                        // The original code commented this out, so we keep it that way for consistency,
                        // but it means accumulated_text_response_ won't contain the full text.
                        // accumulated_text_response_ += text_part;
                        publishFeedback("LLM: " + text_part);
                        accumulated_text_response_ += text_part;
                    }

                    if (delta.contains("tool_calls")) {
                        for (const auto& tool_call_part : delta["tool_calls"]) {
                            int index = tool_call_part["index"];
                            if (accumulated_tool_calls_.size() <= index) {
                                accumulated_tool_calls_.push_back(json::object());
                            }
                            // Ensure the "function" key exists before accessing its sub-keys
                            if (!accumulated_tool_calls_[index].contains("function")) {
                                accumulated_tool_calls_[index]["function"] = json::object();
                            }
                            if (tool_call_part["function"].contains("name")) {
                                accumulated_tool_calls_[index]["function"]["name"] = tool_call_part["function"]["name"];
                            }
                            if (tool_call_part["function"].contains("arguments")) {
                                if (!accumulated_tool_calls_[index]["function"].contains("arguments")) {
                                    accumulated_tool_calls_[index]["function"]["arguments"] = "";
                                }
                                accumulated_tool_calls_[index]["function"]["arguments"] = accumulated_tool_calls_[index]["function"]["arguments"].get<std::string>() + tool_call_part["function"]["arguments"].get<std::string>();
                            }
                        }
                    }
                }
            } catch (const json::exception& e) {
                ROS_WARN_STREAM("JSON parsing error in stream chunk: " << e.what() << " | Chunk: " << data_str);
            }
        }
    }
    return true; 
}

void LLMProcessorNode::nlpCommandCallback(const std_msgs::String::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(_command_mutex); // 保证线程安全
    _last_command = msg->data;
    _new_command_received = true;
    ROS_INFO_STREAM("Queued new NLP command: '" << _last_command << "'");

}
void LLMProcessorNode::processingLoop() {
    ros::Rate rate(5); // 每秒检查5次新命令
    while(ros::ok()) {
        std::string command_to_process;
        bool should_process_new_command = false;
        { 
            std::lock_guard<std::mutex> lock(_command_mutex);
            if (_new_command_received) {
                if (_is_task_executing) {
                    ROS_WARN("A new command was received, but the drone is still executing a previous plan. Ignoring new command.");
                    publishFeedback("Drone is busy. Please wait for the current plan to finish.");
                    // (可选) 可以清除新指令，或者让它排队
                    _new_command_received = false; 
                } else {
                    command_to_process = _last_command;
                    should_process_new_command = true;
                    _new_command_received = false;
                }
            }
        }
        if (should_process_new_command) {
            ROS_INFO_STREAM("Processing command from thread: '" << command_to_process << "'");

    // Publish a clear session start marker
    publishFeedback("LLM_STREAM_START"); 
    
    accumulated_reasoning_response_.clear();
    accumulated_text_response_.clear();
    accumulated_tool_calls_ = json::array();
    
// Dynamically build POI information for the system prompt
std::string poi_info_str = "";

if (!_pois.empty()) {
    poi_info_str += "You have access to the following environmental information:\n";
                
                std::string current_drone_status_copy;
                DroneState current_drone_state_copy;
                geometry_msgs::PoseStamped current_pose_copy;
                {
                    std::lock_guard<std::mutex> lock_status(_status_mutex);
                    current_drone_status_copy = _drone_status;
                    current_drone_state_copy = _drone_state;
                }
                {
                    std::lock_guard<std::mutex> lock_pose(_pose_mutex);
                    current_pose_copy = _current_pose;
                }

                poi_info_str += "The drone's current state is: " + current_drone_status_copy + "\n";
                poi_info_str += "Your current position is (x:" + std::to_string(current_pose_copy.pose.position.x) + 
                                ", y:" + std::to_string(current_pose_copy.pose.position.y) + 
                                ", z:" + std::to_string(current_pose_copy.pose.position.z) + ").\n\n";
    
    poi_info_str += "--- Points of Interest (POIs) ---\n";
    for (const auto& pair : _pois) {
        const POIInfoData& poi = pair.second;
        poi_info_str += "- Name: " + poi.name + "\n";
        poi_info_str += "  Type: " + poi.type + "\n";
        poi_info_str += "  Description: " + poi.description + "\n";
        poi_info_str += "  Reference Coordinates: (x:" + std::to_string(poi.position.x) + 
                        ", y:" + std::to_string(poi.position.y) + 
                        ", z:" + std::to_string(poi.position.z) + ")\n";

               // 向LLM提供返航点信息 >>>
    if (_has_recorded_launch_position) {
     const double rtl_hover_altitude = 2.5; // 定义一个标准的返航悬停高度
    poi_info_str += "The designated Return-to-Launch (RTL) point is (x:" + std::to_string(_launch_position.x) +
                    ", y:" + std::to_string(_launch_position.y) +
                    ", z:" + std::to_string(rtl_hover_altitude) + ").\n\n"; // 使用标准高度
    } else {
      poi_info_str += "The drone is on the ground. The launch point will be recorded upon takeoff.\n\n";
                }
               
        // Present the candidate points for the LLM to choose from
    if (!poi.candidate_points.empty()) {
        poi_info_str += "  **ACTIONABLE GOALS:**\n"; // 措辞更强硬
        for (const auto& p : poi.candidate_points) {
            poi_info_str += "    - " + p.name + ": (x:" + std::to_string(p.point.x) + 
                            ", y:" + std::to_string(p.point.y) + 
                            ", z:" + std::to_string(p.point.z) + ")\n";
        }
    } else {
        // 如果没有候选点，就告诉LLM参考坐标是它唯一能用的目标,实际应用时要修改
        //    // 生产环境的逻辑
    //poi_info_str += "  Safe Approach Point: None defined. This POI cannot be a navigation target.\n";
        poi_info_str += "  **ACTIONABLE GOAL (Reference Point):** (x:" + std::to_string(poi.position.x) + 
                        ", y:" + std::to_string(poi.position.y) + 
                        ", z:" + std::to_string(poi.position.z) + ")\n";
    }

        // Present the simplified boundary for collision avoidance planning
        if (poi.has_simplified_boundary) {
            poi_info_str += "  Obstacle Boundary (min_xy to max_xy): [(" +
                            std::to_string(poi.simplified_boundary.min.x) + ", " +
                            std::to_string(poi.simplified_boundary.min.y) + "), (" +
                            std::to_string(poi.simplified_boundary.max.x) + ", " +
                            std::to_string(poi.simplified_boundary.max.y) + ")]\n";
        }
        poi_info_str += "\n";
    }
} else {
    poi_info_str = "No specific Points of Interest are currently defined.\n";
}

// 拼接最终的系统提示
std::string final_system_prompt = system_prompt_content_ + "\n" + poi_info_str;
   
            // 2. 更新历史记录中的系统Prompt (第一条消息)
            conversation_history_[0]["content"] = final_system_prompt;

            // 3. 将当前用户指令添加到历史记录
            conversation_history_.push_back({
                {"role", "user"},
                {"content", command_to_process}
            });

            // 4. 构建API请求体
            json request_body = {
                {"model", "deepseek-reasoner"},
                {"messages", conversation_history_}, // <<< 使用历史记录
                {"tools", getToolDefinitions()},
                {"tool_choice", "auto"},
                {"stream", true}
                // temperature 被移除了，因为reasoner模型不支持
            };
    
    // The lambda signature now perfectly matches the compiler's expectation for cpr::WriteCallback.
    cpr::WriteCallback write_callback{[this](const std::string_view& data, intptr_t userdata) -> bool {
        (void)userdata; // Suppress unused parameter warning
        return this->streamCallback(std::string(data)); // Convert std::string_view to std::string
    }};

    cpr::Response r = cpr::Post(
        cpr::Url{"https://api.deepseek.com/chat/completions"},
        cpr::Header{{"Authorization", "Bearer " + api_key_},
                    {"Content-Type", "application/json"}},
        cpr::Body{request_body.dump()},
        write_callback,
        cpr::Proxies{{"https", "http://127.0.0.1:7897"}} 
    );

    if (r.status_code == 200) {
        ROS_INFO("Stream finished.");

        if (!accumulated_tool_calls_.empty()) {
            ROS_INFO_STREAM("Total tool calls accumulated: " << accumulated_tool_calls_.size());
            for (const auto& tool_call : accumulated_tool_calls_) {
_tool_call_queue.push_back(tool_call);
                
     }
     // 启动执行流程
     executeNextToolInQueue();
        } 
        else {
            ROS_WARN("Stream ended but no content or tool call was produced.");
            publishFeedback("LLM finished without a clear action.");
            _is_task_executing = false;
        }

    } else {
    conversation_history_.erase(conversation_history_.end() - 1);
        ROS_ERROR_STREAM("LLM API request failed with status " << r.status_code << ": " << r.text);
        publishFeedback("Error: LLM API request failed. Status: " + std::to_string(r.status_code));
    }
        }
        // --- 第二部分：状态驱动的任务推进 ---
        // 检查当前是否有任务正在执行，并且无人机是否已进入可以执行下一步的状态
        DroneState current_drone_state_for_check;
        {
            std::lock_guard<std::mutex> lock_status(_status_mutex);
            current_drone_state_for_check = _drone_state;
        }
        
        if (_is_task_executing && (current_drone_state_for_check == HOVERING)) {
            // 获取最新的 _drone_status
            std::string current_drone_status_for_log;
            {
                std::lock_guard<std::mutex> lock_status(_status_mutex);
                current_drone_status_for_log = _drone_status;
            }
            ROS_INFO_STREAM("Previous action seems complete (Drone is idle). Checking for next action in queue. Current state: " << current_drone_status_for_log);

            executeNextToolInQueue();
        }
        rate.sleep();
    }
}

json LLMProcessorNode::getToolDefinitions() {
    return json::parse(R"([
        {
            "type": "function",
            "function": {
                "name": "takeoff",
                "description": "Initiates the takeoff sequence to a standard hover altitude. Use to start a flight.",
                "parameters": {"type": "object", "properties": {}} 
            }
        },
        { 
            "type": "function",
            "function": {
                "name": "land",
                "description": "Initiates the landing sequence from the current position.",
                "parameters": {"type": "object", "properties": {}} 
            }
        },
        {
            "type": "function",
            "function": {
                "name": "go_to_waypoint",
                "description": "Commands the drone to fly to a specific 3D coordinate (x, y, z) in meters and then hover. Drone must be in HOVERING or FLYING state.",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "x": { "type": "number", "description": "Target X coordinate in meters." },
                        "y": { "type": "number", "description": "Target Y coordinate in meters." },
                        "z": { "type": "number", "description": "Target Z coordinate in meters." }
                    },
                    "required": ["x", "y", "z"]
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "inspection_sequence",
                "description": "After arriving at an observation point, this function commands the drone to first turn and face a specified target POI, back up slightly, and then perform a comprehensive visual inspection (camera up/down, drone rotation left/right).",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "target_name": {
                            "type": "string",
                            "description": "The exact name of the Point of Interest (e.g., 'House_1', 'Solar_Panel_Array') that the drone should turn to face before starting the inspection."
                        }
                    },
                    "required": ["target_name"]
                }
            }
        },
        {
    "type": "function",
    "function": {
        "name": "wait",
        "description": "Commands the drone to hover at its current position for a specified number of seconds. This is a non-navigation action.",
        "parameters": {
            "type": "object",
            "properties": {
                "duration_seconds": {
                    "type": "number",
                    "description": "The duration to hover in seconds."
                }
            },
            "required": ["duration_seconds"]
        }
    }
}   
    ])");
}

void LLMProcessorNode::publishFeedback(const std::string& text) {
    std_msgs::String msg;
    msg.data = text;
    nlp_feedback_pub_.publish(msg);
}
void LLMProcessorNode::currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(_pose_mutex); 
    _current_pose = *msg; 
}

void LLMProcessorNode::publishGoal(const geometry_msgs::PoseStamped& goal_pose, const std::string& action_name) {
    // 检查是否有节点在监听
    ros::Rate poll_rate(10);
    while(llm_goal_pub_.getNumSubscribers() == 0) {
        ROS_INFO_ONCE("Waiting for A* planner to subscribe to /llm_goal...");
        if (!ros::ok()) return;
        poll_rate.sleep();
    }
    
    llm_goal_pub_.publish(goal_pose);
    ROS_INFO_STREAM("Published goal for action '" << action_name << "' to /llm_goal topic.");
}
std::string LLMProcessorNode::takeoff_action() {
    DroneState current_drone_state_copy;
    {
        std::lock_guard<std::mutex> lock_status(_status_mutex);
        current_drone_state_copy = _drone_state;
    }
    if (current_drone_state_copy != GROUNDED) {
        return "Error: Drone is not on the ground. Cannot take off.";
    }
    if (!_has_recorded_launch_position) {
        geometry_msgs::Point current_pos_copy;
        {
            std::lock_guard<std::mutex> lock_pose(_pose_mutex);
            current_pos_copy = _current_pose.pose.position;
        }
        _launch_position = current_pos_copy;
        _has_recorded_launch_position = true;
        ROS_INFO_STREAM("Launch position recorded at: (" 
                        << _launch_position.x << ", " 
                        << _launch_position.y << ", " 
                        << _launch_position.z << ")");
    }
    nlp_drone_control::ExecuteDroneAction srv;
    srv.request.action_type = "takeoff";
    
    // 我们可以通过JSON传递参数，比如起飞高度
    json params = { {"altitude", 3.0} };
    srv.request.params_json = params.dump();

    if (execute_action_client_.call(srv) && srv.response.success) {
        // 服务调用成功启动了Unity端的协程
        // 我们不需要在这里等待，状态机将依赖 /drone_status 的更新来推进
        return "Takeoff command sent to Unity successfully.";
    } else {
        // 如果服务调用失败，这是一个严重的错误
        std::string error_msg = "Error: Failed to execute takeoff action via service call.";
        ROS_ERROR_STREAM(error_msg);
        return error_msg;
    }
}

std::string LLMProcessorNode::land_action() {
    DroneState current_drone_state_copy;
    {
        std::lock_guard<std::mutex> lock_status(_status_mutex);
        current_drone_state_copy = _drone_state;
    }
    if (current_drone_state_copy == GROUNDED) {
        return "Error: Drone is already on the ground.";
    }

    nlp_drone_control::ExecuteDroneAction srv;
    srv.request.action_type = "land";
    srv.request.params_json = "{}"; // 降落不需要额外参数

    if (execute_action_client_.call(srv) && srv.response.success) {
        return "Land command sent to Unity successfully.";
    } else {
        std::string error_msg = "Error: Failed to execute land action via service call.";
        ROS_ERROR_STREAM(error_msg);
        return error_msg;
    }
}
std::string LLMProcessorNode::wait_action(double duration) {
    if (duration <= 0) {
        return "Error: Wait duration must be positive.";
    }
    
    // 检查无人机是否在空中
    DroneState current_drone_state_copy;
    {
        std::lock_guard<std::mutex> lock_status(_status_mutex);
        current_drone_state_copy = _drone_state;
    }
    if (current_drone_state_copy == GROUNDED) {
        return "Error: Drone is on the ground. Cannot execute wait action.";
    }

    ROS_INFO_STREAM("Sending wait command to Unity for " << duration << " seconds.");

    nlp_drone_control::ExecuteDroneAction srv;
    srv.request.action_type = "wait"; 
    srv.request.params_json = json{{"duration", duration}}.dump();

    if (execute_action_client_.call(srv) && srv.response.success) {
        return "Wait command sent to Unity successfully.";
    } else {
        std::string error_msg = "Error: Failed to execute wait action via service call.";
        ROS_ERROR_STREAM(error_msg);
        return error_msg;
    }
}

std::string LLMProcessorNode::go_to_waypoint_action(double target_x, double target_y, double target_z) {
    DroneState current_drone_state_copy;
    {
        std::lock_guard<std::mutex> lock_status(_status_mutex);
        current_drone_state_copy = _drone_state;
    }

    if (current_drone_state_copy != HOVERING) {
        return "Error: Drone must be in a stable HOVERING state to start navigation. Current state is not HOVERING.";
    }
    // 检查2 (新增)：是否正在执行另一个动作
    if (current_drone_state_copy == PERFORMING_ACTION) {
        std::string msg = "Error: Drone is currently busy performing another action (e.g., rotating, adjusting camera). Please wait for it to finish before sending a navigation command.";
        publishFeedback(msg);
        return msg;
    }
    
    // (可选) 检查3：是否已经在导航中
    if (current_drone_state_copy == FLYING) {
        ROS_WARN("Drone is already flying to a target. The new waypoint will override the current one.");
    }
    // _drone_state = FLYING; // 这一行是写入操作，也需要加锁
    {
        std::lock_guard<std::mutex> lock_status(_status_mutex);
        _drone_state = FLYING; 
    }

    geometry_msgs::PoseStamped goal_pose;
    goal_pose.header.stamp = ros::Time::now();
    goal_pose.header.frame_id = "map";

    goal_pose.pose.position.x = target_x;
    goal_pose.pose.position.y = target_y;
    goal_pose.pose.position.z = target_z;

    // For orientation, we can either keep the current one or make it face the goal.
    // Let's keep it simple and just set a default orientation. The A* planner or Unity can handle turning.
    goal_pose.pose.orientation.w = 1.0; 

    publishGoal(goal_pose, "go_to_waypoint");

    std::string result_msg = "Goal (" + std::to_string(target_x) + ", " + std::to_string(target_y) + ", " + std::to_string(target_z) + ") sent to A* planner.";
    //publishFeedback(result_msg);
    return result_msg;

}

std::string LLMProcessorNode::adjust_camera_action(double pitch_angle) {
    ROS_INFO_STREAM("Internal call: Adjusting camera pitch to " << pitch_angle);
    nlp_drone_control::ExecuteDroneAction srv;
    srv.request.action_type = "adjust_camera_pitch"; 
    srv.request.params_json = json{{"pitch_angle", pitch_angle}}.dump();

    if (execute_action_client_.call(srv) && srv.response.success) {
        return "Success";
    }
    return "Error: Failed to execute adjust_camera_pitch action.";
}

bool LLMProcessorNode::rotate_yaw_action(double degrees) {
    ROS_INFO_STREAM("Internal call: Rotating drone yaw by " << degrees << " degrees.");
    nlp_drone_control::ExecuteDroneAction srv;
    srv.request.action_type = "rotate_drone_yaw_relative"; 
    srv.request.params_json = json{{"angle_degrees", degrees}}.dump();

    if (execute_action_client_.call(srv) && srv.response.success) {
        return true;
    }
    ROS_ERROR_STREAM("Failed to execute rotate_drone_yaw_relative action for " << degrees << " degrees.");
    return false;
}

// 执行队列中的下一个任务
void LLMProcessorNode::executeNextToolInQueue() {
    // 如果队列为空，说明整个计划已完成
    if (_tool_call_queue.empty()) {
        if (_is_task_executing) {
            ROS_INFO("--- Task queue is empty. Plan execution finished successfully. ---");
            publishFeedback("Plan finished.");
            _is_task_executing = false; // 任务结束，系统变为空闲
        }
        return;
    }

    // 只有在无人机空闲时才执行下一步
    DroneState current_drone_state_for_check;
    std::string current_drone_status_for_log;
    {
        std::lock_guard<std::mutex> lock_status(_status_mutex);
        current_drone_state_for_check = _drone_state;
        current_drone_status_for_log = _drone_status;
    }
        if (current_drone_state_for_check != HOVERING && current_drone_state_for_check != GROUNDED) {
        ROS_INFO_STREAM("Waiting for drone to become idle before executing next step. Current state: " << current_drone_status_for_log);
        return;
    }

    // 从队列头部取出一个任务
    json next_tool = _tool_call_queue.front();
    _tool_call_queue.pop_front();

    ROS_INFO("--- Executing next step from queue ---");
    _is_task_executing = true; // 标记任务开始执行

    // 调用封装好的执行函数
    std::string result_msg = executeTool(next_tool);

    // 检查执行结果。如果工具函数因为状态不符等原因立即返回了错误，
    // 我们应该停止整个计划。
    if (result_msg.rfind("Error:", 0) == 0) {
        ROS_ERROR_STREAM("Stopping plan execution due to an error: " << result_msg);
        publishFeedback("Plan failed: " + result_msg);
        _tool_call_queue.clear(); // 清空队列
        _is_task_executing = false; // 结束执行
    } 
}

std::string LLMProcessorNode::executeTool(const json& tool_call) {
    std::string tool_name;
    json tool_input;

    try {
        tool_name = tool_call.at("function").at("name").get<std::string>();
        tool_input = json::parse(tool_call.at("function").at("arguments").get<std::string>());
    } catch (const json::exception& e) {
        std::string error_msg = "Error: Failed to parse tool call JSON: " + std::string(e.what());
        ROS_ERROR_STREAM(error_msg);
        publishFeedback(error_msg);
        return error_msg;
    }
    
    ROS_INFO_STREAM("Executing tool: " << tool_name << " with input: " << tool_input.dump());
    std::string result_msg = "";

    if (tool_name == "takeoff") {
        result_msg = takeoff_action();
    } else if (tool_name == "land") {
        result_msg = land_action();
    } else if (tool_name == "go_to_waypoint") {
        if (tool_input.contains("x") && tool_input.contains("y") && tool_input.contains("z")) {
            result_msg = go_to_waypoint_action(
                tool_input["x"].get<double>(),
                tool_input["y"].get<double>(),
                tool_input["z"].get<double>()
            );
        } else {
            result_msg = "Error: Missing x, y, or z parameters for go_to_waypoint.";
        }
    }else if (tool_name == "wait") {
        if (tool_input.contains("duration_seconds")) {
            result_msg = wait_action(tool_input["duration_seconds"].get<double>());
        } else {
            result_msg = "Error: 'duration_seconds' is missing for wait tool.";
        }
    } else if (tool_name == "inspection_sequence") {
        if (tool_input.contains("target_name")) {
            std::string target = tool_input["target_name"].get<std::string>();
            result_msg = perform_visual_inspection_action(target); // <-- Pass the target name
        } else {
            result_msg = "Error: 'target_name' parameter is missing for inspection_sequence.";
        }
    } else {
        result_msg = "Error: Unknown or unsupported tool: " + tool_name;
    }
    
    // 如果工具函数内部执行成功，发布其返回的消息
    if (result_msg.rfind("Error:", 0) != 0) {
        publishFeedback("LLM Action: " + result_msg);
    }
    
    return result_msg;
}
std::string LLMProcessorNode::perform_visual_inspection_action(const std::string& target_name) {
    DroneState current_drone_state_copy;
    {
        std::lock_guard<std::mutex> lock_status(_status_mutex);
        current_drone_state_copy = _drone_state;
    }
    if (current_drone_state_copy != HOVERING) {
        return "Error: Drone must be in a stable HOVERING state for visual inspection.";
    }
    // --- 步骤 0: 验证目标POI是否存在 ---
    if (_pois.find(target_name) == _pois.end()) {
        return "Error: The specified target '" + target_name + "' is not a known Point of Interest.";
    }
    geometry_msgs::Point target_pos = _pois[target_name].position;
    publishFeedback("Starting targeted inspection of '" + target_name + "'.");

    // --- 辅助函数Lambda表达式 (保持不变，用于等待) ---
    auto wait_and_spin = [&](double duration_sec) {
        ros::Time start_time = ros::Time::now();
        ros::Rate rate(50);
        while (ros::ok() && (ros::Time::now() - start_time < ros::Duration(duration_sec))) {
            ros::spinOnce();
            rate.sleep();
        }
    };

    auto waitForActionCompletion = [&]() -> bool {
        {
            std::lock_guard<std::mutex> lock(_feedback_mutex);
            _action_completed = false; // Reset the flag before waiting
        }
        ros::Time start_time = ros::Time::now();
        ros::Rate rate(20);
        while (ros::ok()) {
            {
                std::lock_guard<std::mutex> lock(_feedback_mutex);
                if (_action_completed) {
                    return true;
                }
            }
            if (ros::Time::now() - start_time > ros::Duration(15.0)) { // Using existing timeout
                ROS_ERROR("Timeout: Did not receive 'ACTION_COMPLETE' from Unity in time.");
                return false;
            }
            ros::spinOnce();
            rate.sleep();
        }
        return false;
    };
    
    //用于封装动作调用和等待
    auto execute_and_wait = [&](const std::function<bool()>& action_func, const std::string& step_name) -> bool {
        ROS_INFO_STREAM("Executing step: " << step_name);
        if (!action_func()) {
            ROS_ERROR_STREAM("Failed to send command for step: " << step_name);
            return false;
        }
        if (!waitForActionCompletion()) {
            ROS_ERROR_STREAM("Timeout waiting for completion of step: " << step_name);
            return false;
        }
        ROS_INFO_STREAM("Step '" << step_name << "' completed. Pausing briefly.");
        wait_and_spin(1.0); // 动作完成后短暂等待，让系统稳定
        return true;
    };
    // ★★★ 新增：稳定延迟步骤 ★★★
    // 在开始任何精确计算之前，让无人机稳定悬停一段时间。
    const double stabilization_delay = 2.0; // seconds,可以根据实际情况调整
    publishFeedback("Arrived at observation point. Stabilizing for " + std::to_string(stabilization_delay) + " seconds...");
    wait_and_spin(stabilization_delay);
    publishFeedback("Stabilization complete. Starting inspection sequence.");
    // ★★★ 延迟结束 ★★★
    
// --- 步骤 1: 初始转向，让无人机朝向目标POI (最终修正版) ---
publishFeedback("Phase 1/3: Orienting towards target...");

// 获取快照 (不变)
geometry_msgs::PoseStamped current_pose_snapshot;
{
    std::lock_guard<std::mutex> lock_pose(_pose_mutex);
    current_pose_snapshot = _current_pose;
}
geometry_msgs::Point current_pos_ros = current_pose_snapshot.pose.position;

// === 在ROS坐标系中准备向量 ===
// 1. 无人机当前朝向向量 (ROS +X 轴是前方)
tf2::Quaternion q_ros;
tf2::fromMsg(current_pose_snapshot.pose.orientation, q_ros);
tf2::Vector3 forward_vec_ros_3d = tf2::Transform(q_ros) * tf2::Vector3(1, 0, 0);

// 2. 目标方向向量
tf2::Vector3 target_dir_ros_3d(target_pos.x - current_pos_ros.x,
                               target_pos.y - current_pos_ros.y,
                               target_pos.z - current_pos_ros.z); // 计算完整3D向量

// ★★★ 关键修正 1: 在ROS的XY水平面上进行投影和归一化 ★★★
tf2::Vector3 forward_vec_ros_2d = forward_vec_ros_3d;
forward_vec_ros_2d.setZ(0); // 投影到XY平面
if (forward_vec_ros_2d.length() < 0.001) {
    // 处理无人机垂直朝上的特殊情况
    ROS_WARN("Drone is pointing vertically. Using a default forward direction for yaw calculation.");
    forward_vec_ros_2d.setValue(1.0, 0.0, 0.0);
}
forward_vec_ros_2d.normalize();

tf2::Vector3 target_dir_ros_2d = target_dir_ros_3d;
target_dir_ros_2d.setZ(0); // 投影到XY平面
if (target_dir_ros_2d.length() < 0.001) {
    ROS_ERROR("Target is directly above or below the drone. Cannot determine a unique yaw direction.");
    return "Inspection failed: Target is vertically aligned, cannot orient.";
}
target_dir_ros_2d.normalize();


// === 现在，我们只在2D平面上计算角度，这样更简单直接 ===
// ★★★ 关键修正 2: 直接在ROS 2D平面上计算角度差 ★★★

// ROS中，atan2(y, x) 给出与+X轴的夹角，逆时针为正
double current_yaw_rad_ros = atan2(forward_vec_ros_2d.y(), forward_vec_ros_2d.x());
double target_yaw_rad_ros  = atan2(target_dir_ros_2d.y(), target_dir_ros_2d.x());

double delta_yaw_rad_ros = target_yaw_rad_ros - current_yaw_rad_ros;

// 将角度规范化到 [-PI, PI]
while (delta_yaw_rad_ros > M_PI)  delta_yaw_rad_ros -= 2.0 * M_PI;
while (delta_yaw_rad_ros <= -M_PI) delta_yaw_rad_ros += 2.0 * M_PI;

// ★★★ 关键修正 3: 将ROS的角度差转换为Unity的API调用 ★★★
// ROS中，逆时针为正。根据我们之前的实验，Unity API需要顺时针为正。
// 因此，我们需要取反。
double unity_api_angle_deg = - (delta_yaw_rad_ros * 180.0 / M_PI);

ROS_INFO_STREAM("ROS Yaw Diff (rad): " << delta_yaw_rad_ros << ", Unity API Command (deg): " << unity_api_angle_deg);

// 后续逻辑不变
if (std::abs(unity_api_angle_deg) > 5.0) {
    if (!execute_and_wait([&](){ return rotate_yaw_action(unity_api_angle_deg); }, "Initial Orient")) {
        return "Inspection failed at initial orientation step.";
    }
} else {
    publishFeedback("Already facing target, skipping initial orientation.");
}

    // --- 步骤 2: 执行后退 ---
    publishFeedback("Phase 2/3: Backing up for a better view...");
    if (!execute_and_wait([&](){ return backup_action(2.0); }, "Backup")) {
        return "Inspection failed at backup step.";
    }

    // --- 步骤 3: 执行纯粹相对旋转的拍照序列---
    publishFeedback("Phase 3/3: Performing multi-angle visual scan...");

    // 准备图像收集和文件保存
    std::vector<sensor_msgs::Image> collected_images;
    sensor_msgs::Image current_image;
    
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
    std::string timestamp_folder = ss.str();
    std::string base_save_path = "/home/lpf/docker_shared/rltraining_ws/src/nlp_drone_control/images";
    std::string save_directory = base_save_path + "/" + timestamp_folder;
    publishFeedback("Photos will be saved to: " + save_directory);
    
    auto capture_photo = [&](const std::string& filename) -> bool {
        if (!capture_and_get_image(save_directory, filename, current_image)) {
            return false;
        }
        collected_images.push_back(current_image);
        return true;
    };

    const double INSPECTION_YAW_ANGLE = 60.0; // 定义检查角度为60度

    // 3.1: 调整相机并拍照 (上/下/前)
    publishFeedback("Step 1/5: Scanning UP/DOWN/FORWARD...");
    if (!execute_and_wait([&](){ return adjust_camera_action(-45.0) == "Success"; }, "Tilt Up")) return "Inspection failed.";
    if (!capture_photo("0_up.jpg")) return "Inspection failed.";
    
    if (!execute_and_wait([&](){ return adjust_camera_action(45.0) == "Success"; }, "Tilt Down")) return "Inspection failed.";
    if (!capture_photo("1_down.jpg")) return "Inspection failed.";

    if (!execute_and_wait([&](){ return adjust_camera_action(0.0) == "Success"; }, "Level Camera")) return "Inspection failed.";
    if (!capture_photo("2_forward.jpg")) return "Inspection failed.";

    // 3.2: 从当前位置向左转60度，然后拍照
    publishFeedback("Step 2/5: Rotating LEFT for photo...");
    // 在Unity中，相对旋转正角度是逆时针（向左）
    if (!execute_and_wait([&](){ return rotate_yaw_action(-INSPECTION_YAW_ANGLE); }, "Rotate Left")) return "Inspection failed.";
    if (!capture_photo("3_left.jpg")) return "Inspection failed.";
    
    // 3.3: 从当前位置向右转120度，然后拍照 
    publishFeedback("Step 3/5: Rotating RIGHT for photo...");
    // 在Unity中，相对旋转负角度是顺时针（向右）
    if (!execute_and_wait([&](){ return rotate_yaw_action(2.0 * INSPECTION_YAW_ANGLE); }, "Rotate Right")) return "Inspection failed.";
    if (!capture_photo("4_right.jpg")) return "Inspection failed.";

    publishFeedback("Step 4/5: Returning to center after inspection...");
    if (!execute_and_wait([&](){ return rotate_yaw_action(-INSPECTION_YAW_ANGLE); }, "Return to Center")) return "Inspection failed.";
    
    publishFeedback("Step 5/5: All images captured. Requesting comprehensive description from VLM...");
    
    vlm_service::DescribeScene srv;
    srv.request.images = collected_images;
    srv.request.prompt = "你收到了从无人机同一位置、多角度拍摄的五张照片(上、下、前、左、右)。请综合分析所有信息，提供一个关于当前场景的全面描述，重点关注设备状态或潜在异常。";
    
    if (describe_scene_client_.call(srv) && srv.response.success) {
        return "Visual Inspection Report: " + srv.response.description;
    } else {
        return "Error: Failed to get a comprehensive description from the VLM service.";
    }
}
bool LLMProcessorNode::capture_and_get_image(const std::string& dir, const std::string& filename, sensor_msgs::Image& out_image) {
    vlm_service::CaptureImage srv;
    srv.request.directory_path = dir;
    srv.request.file_name = filename;
    
    if (capture_image_client_.call(srv) && srv.response.success) {
        ROS_INFO_STREAM("Successfully captured image '" << filename << "' via service. It was saved to " << srv.response.file_path);
        out_image = srv.response.captured_image; // 从响应中获取图像数据
        ros::Duration(0.2).sleep(); // 短暂等待，确保动作和拍照之间有间隔
        return true;
    } else {
        ROS_ERROR_STREAM("Failed to capture image '" << filename << "'. Service message: " << srv.response.message);
        return false;
    }
}
// 在您的类中添加一个辅助函数
tf2::Vector3 LLMProcessorNode::transformRosToUnity(const tf2::Vector3& ros_vec) {
    return tf2::Vector3(
        -ros_vec.y(), // Unity X is ROS -Y
         ros_vec.z(), // Unity Y is ROS +Z
         ros_vec.x()  // Unity Z is ROS +X
    );
}
bool LLMProcessorNode::backup_action(double distance) {
    ROS_INFO_STREAM("Internal call: Backing up by " << distance << " meters.");
    nlp_drone_control::ExecuteDroneAction srv;
    srv.request.action_type = "backup";
    srv.request.params_json = json{{"distance", distance}}.dump();

    if (execute_action_client_.call(srv) && srv.response.success) {
        return true;
    }
    ROS_ERROR_STREAM("Failed to execute backup action for " << distance << " meters.");
    return false;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "llm_processor_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2); // 使用2个线程。一个处理长任务，一个处理短回调
    spinner.start();
    LLMProcessorNode node(nh); 
    ros::waitForShutdown();
    return 0;
}
