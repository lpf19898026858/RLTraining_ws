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

    // 5. 创建中央接口
    describe_scene_client_ = nh_.serviceClient<vlm_service::DescribeScene>("/vision_service/describe_scene");
    central_nlp_command_sub_ = nh_.subscribe("/nlp_command", 1, &LLMProcessorNode::centralNlpCommandCallback, this);
    central_nlp_feedback_pub_ = nh_.advertise<std_msgs::String>("/nlp_feedback", 10);
    capture_image_client_ = nh_.serviceClient<vlm_service::CaptureImage>("/vision_service/capture_image");
    
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
    last_command_ = msg->data;
    new_command_received_ = true;
    //ROS_INFO_STREAM("Queued new FLEET command: '" << last_command_ << "'");
}

void LLMProcessorNode::dronePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, const std::string& drone_id) {
    std::lock_guard<std::mutex> lock(contexts_map_mutex_);
    if (drone_contexts_.count(drone_id)) drone_contexts_.at(drone_id).pose = *msg;
}

void LLMProcessorNode::droneStatusCallback(const std_msgs::String::ConstPtr& msg, const std::string& drone_id) {
    std::lock_guard<std::mutex> lock(contexts_map_mutex_);
    if (drone_contexts_.count(drone_id)) {
        auto& context = drone_contexts_.at(drone_id);
        context.status_str = msg->data;
        if (context.status_str == "IDLE_ON_GROUND") context.state_enum = GROUNDED;
        else if (context.status_str == "NAVIGATING") context.state_enum = FLYING;
        else if (context.status_str == "HOVERING" || context.status_str == "IDLE_IN_AIR") context.state_enum = HOVERING;
        else if (context.status_str == "PERFORMING_ACTION") context.state_enum = PERFORMING_ACTION;
        else if (context.status_str == "TAKING_OFF") context.state_enum = TAKING_OFF;
        else if (context.status_str == "LANDING") context.state_enum = LANDING;
        else context.state_enum = UNKNOWN;
    }
}

void LLMProcessorNode::actionFeedbackCallback(const std_msgs::String::ConstPtr& msg, const std::string& drone_id) {
    if (drone_contexts_.count(drone_id)) {
        auto& context = drone_contexts_.at(drone_id);
        std::lock_guard<std::mutex> lock(context.context_data_mutex);
        if (msg->data == "ACTION_COMPLETE") context.action_completed = true;
    }
}

bool LLMProcessorNode::streamCallback(const std::string& chunk) {
    std::stringstream ss(chunk);
    std::string line;

    while (std::getline(ss, line)) {
        if (line.rfind("data: ", 0) == 0) {
            std::string data_str = line.substr(6);
            
            //ROS_INFO_STREAM("Raw Stream Data: " << data_str);

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


void LLMProcessorNode::processingLoop() {
    ros::Rate rate(5); // 每秒检查5次新命令
    while(ros::ok()) {
        std::string command_to_process;
        bool should_process = false;
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            if (new_command_received_) {
                command_to_process = last_command_;
                should_process = true;
                new_command_received_ = false;
            }
        }

        if (should_process) {
            publishFeedback("LLM_STREAM_START");
            accumulated_tool_calls_ = json::array();
            accumulated_reasoning_response_.clear();
            accumulated_text_response_.clear();
            
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

            std::string final_system_prompt = system_prompt_content_ + "\n" + dynamic_info_str;

            conversation_history_[0]["content"] = final_system_prompt;
            conversation_history_.push_back({{"role", "user"}, {"content", command_to_process}});
            
            trimConversationHistory();

            json request_body = {
                {"model", "deepseek-chat"},
                {"messages", conversation_history_},
                {"tools", getFleetToolDefinitions()},
                {"tool_choice", "auto"},
                {"stream", true}
            };
    
           cpr::WriteCallback write_callback{[this](const std::string_view& data, intptr_t userdata) -> bool {
                return this->streamCallback(std::string(data));
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
        if (!accumulated_tool_calls_.empty()) {
             publishFeedback("Plan received. Enqueueing tasks for drones...");
            dispatchToolCalls(accumulated_tool_calls_);
     }
        else {
		publishFeedback("LLM finished without a clear action."); 
	}
	}
    else {
    conversation_history_.erase(conversation_history_.end() - 1);
        ROS_ERROR_STREAM("LLM API request failed with status " << r.status_code << ": " << r.text);
        publishFeedback("Error: LLM API request failed. Status: " + std::to_string(r.status_code));
    }
   }
        rate.sleep();
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
void LLMProcessorNode::dispatchToolCalls(const json& tool_calls) {
    std::lock_guard<std::mutex> lock(contexts_map_mutex_); // 锁定整个 map 以安全地访问内部 context
    for (const auto& tool_call : tool_calls) {
        try {
            json args = json::parse(tool_call.at("function").at("arguments").get<std::string>());
            std::string drone_id = args.at("drone_id").get<std::string>();

            // 检查无人机ID是否存在
            if (drone_contexts_.count(drone_id)) {
                // 获取对应无人机的上下文，并锁定其内部数据
                auto& context = drone_contexts_.at(drone_id);
                std::lock_guard<std::mutex> context_lock(context.context_data_mutex);
                
                // 将任务放入该无人机专属的队列
                context.tool_call_queue.push_back(tool_call);
                ROS_INFO_STREAM("Enqueued task '" << tool_call.at("function").at("name") 
                                << "' for drone " << drone_id);
            } else {
                ROS_ERROR_STREAM("Cannot enqueue task for unknown drone_id: " << drone_id);
            }
        } catch (const std::exception& e) {
            ROS_ERROR("Error parsing or enqueuing tool call: %s. Full call: %s", e.what(), tool_call.dump().c_str());
        }
    }
}

json LLMProcessorNode::getFleetToolDefinitions() {
    return json::parse(R"END([
        {
            "type": "function",
            "function": {
                "name": "takeoff",
                "description": "命令指定的无人机从地面起飞至标准悬停高度。 (Commands a specific drone to take off from the ground to a standard hover altitude.)",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "drone_id": {
                            "type": "string",
                            "description": "要执行此动作的无人机ID。必须是机队列表中的一个有效ID。 (The ID of the drone to perform this action. Must be a valid ID from the fleet list, e.g., 'V_UAV_0')."
                        }
                    },
                    "required": ["drone_id"]
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "land",
                "description": "命令指定的无人机从当前位置就地降落。 (Commands a specific drone to land at its current position.)",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "drone_id": {
                            "type": "string",
                            "description": "要执行此动作的无人机ID。"
                        }
                    },
                    "required": ["drone_id"]
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "go_to_waypoint",
                "description": "命令指定的无人机飞往一个具体的三维世界坐标点。 (Commands a specific drone to fly to a specific 3D world coordinate.)",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "drone_id": { "type": "string", "description": "要执行此动作的无人机ID。" },
                        "x": { "type": "number", "description": "目标X坐标 (单位：米)。 (Target X coordinate in meters)." },
                        "y": { "type": "number", "description": "目标Y坐标 (单位：米)。 (Target Y coordinate in meters)." },
                        "z": { "type": "number", "description": "目标Z坐标 (单位：米)。 (Target Z coordinate in meters)." }
                    },
                    "required": ["drone_id", "x", "y", "z"]
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "perform_visual_inspection",
                "description": "命令指定的无人机在其当前位置，朝向一个目标POI，执行一次完整的视觉检查扫描序列。 (Commands a specific drone to perform a full visual inspection sequence at its current location, oriented towards a target POI.)",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "drone_id": { "type": "string", "description": "要执行此动作的无人机ID。" },
                        "target_name": {
                            "type": "string",
                            "description": "无人机在开始扫描前需要朝向的POI的精确名称。 (The exact name of the Point of Interest to face before starting the scan, e.g., 'House_1')."
                        }
                    },
                    "required": ["drone_id", "target_name"]
                }
            }
        },
        {
            "type": "function",
            "function": {
                "name": "wait",
                "description": "命令指定的无人机在当前位置悬停指定的秒数。 (Commands a specific drone to hover at its current position for a specified number of seconds.)",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "drone_id": { "type": "string", "description": "要执行此动作的无人机ID。" },
                        "duration_seconds": {
                            "type": "number",
                            "description": "悬停的秒数。 (The duration to hover in seconds)."
                        }
                    },
                    "required": ["drone_id", "duration_seconds"]
                }
            }
        },
        {
    "type": "function",
    "function": {
        "name": "return_to_launch",
        "description": "命令指定的无人机返回其原始起飞点并悬停，为降落做准备。(Commands a specific drone to return to its original launch point and hover, preparing for landing.)",
        "parameters": {
            "type": "object",
            "properties": {
                "drone_id": {
                    "type": "string",
                    "description": "要执行此动作的无人机ID。"
                }
            },
            "required": ["drone_id"]
        }
    }
}
    ])END");
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
    const ros::Duration NAVIGATION_TIMEOUT(60.0); // 设置一个60秒的导航超时
    while (ros::ok()) {
         // --- 1. 检查导航是否超时 ---
        {
            std::lock_guard<std::mutex> lock(contexts_map_mutex_);
            auto& context = drone_contexts_.at(drone_id);
            if (context.state_enum == FLYING && !context.navigation_start_time_.is_zero() &&
                (ros::Time::now() - context.navigation_start_time_ > NAVIGATION_TIMEOUT))
            {
                ROS_ERROR("Drone %s navigation timed out!", drone_id.c_str());
                publishFeedback("Error: Plan for " + drone_id + " aborted due to navigation timeout.");
                
                std::lock_guard<std::mutex> context_lock(context.context_data_mutex);
                context.tool_call_queue.clear(); // 清空任务队列
                context.is_task_executing = false; // 重置执行状态
                
                // (可选但推荐) 发送一个悬停指令作为安全措施
                // nlp_drone_control::ExecuteDroneAction srv;
                // srv.request.action_type = "hover";
                // execute_action_clients_.at(drone_id).call(srv);
                
                context.state_enum = HOVERING; // 强制将状态设置为空闲，以防万一
            }
        }   
        bool is_drone_idle = false;
        bool has_tasks_in_queue = false;
        {
            std::lock_guard<std::mutex> lock(contexts_map_mutex_);
            auto& context = drone_contexts_.at(drone_id);
            std::lock_guard<std::mutex> context_lock(context.context_data_mutex);
            is_drone_idle = (context.state_enum == HOVERING || context.state_enum == GROUNDED) && !context.is_task_executing;
            has_tasks_in_queue = !context.tool_call_queue.empty();
        }

        if (is_drone_idle && has_tasks_in_queue) {
            json next_tool;
            {
                std::lock_guard<std::mutex> lock(contexts_map_mutex_);
                auto& context = drone_contexts_.at(drone_id);
                std::lock_guard<std::mutex> context_lock(context.context_data_mutex);
                next_tool = context.tool_call_queue.front();
                context.tool_call_queue.pop_front();
                context.is_task_executing = true;
            }
            
            std::string result = executeTool(next_tool, drone_id);
            
            {
                std::lock_guard<std::mutex> lock(contexts_map_mutex_);
                auto& context = drone_contexts_.at(drone_id);
                std::lock_guard<std::mutex> context_lock(context.context_data_mutex);
                context.is_task_executing = false;
                if(result.rfind("Error:", 0) == 0) {
                   context.tool_call_queue.clear();
                   publishFeedback("Plan for " + drone_id + " aborted due to error: " + result);
                }
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
std::string LLMProcessorNode::takeoff_action(const std::string& drone_id) {
    // 前提条件检查
    {
        std::lock_guard<std::mutex> lock(contexts_map_mutex_);
        auto& context = drone_contexts_.at(drone_id);
        if (context.state_enum != GROUNDED) return "Error: Drone is not on the ground.";
        if (!context.has_recorded_launch_position) {
            context.launch_position = context.pose.pose.position;
            context.has_recorded_launch_position = true;
        }
    }
    
    if (execute_and_wait_for_completion(drone_id, "takeoff", json{{"altitude", 3.0}}.dump(), "Takeoff")) {
        return "Takeoff successful. Now hovering.";
    }
    
    return "Error: Takeoff action failed or timed out.";
}

std::string LLMProcessorNode::land_action(const std::string& drone_id) {
    // 前提条件检查
    {
        std::lock_guard<std::mutex> lock(contexts_map_mutex_);
        auto& context = drone_contexts_.at(drone_id);
        if (context.state_enum == GROUNDED) return "Error: Drone is already on the ground.";
    }

    if (execute_and_wait_for_completion(drone_id, "land", "{}", "Land")) {
        return "Landing successful. Drone is on the ground.";
    }
    
    return "Error: Land action failed or timed out.";
}

std::string LLMProcessorNode::wait_action(double duration, const std::string& drone_id) {
    // 前提条件检查
    {
        std::lock_guard<std::mutex> lock(contexts_map_mutex_);
        auto& context = drone_contexts_.at(drone_id);
        if (context.state_enum == GROUNDED) return "Error: Drone is on the ground.";
    }

    std::string params = json{{"duration", duration}}.dump();
    if (execute_and_wait_for_completion(drone_id, "wait", params, "Wait")) {
        return "Wait finished.";
    }

    return "Error: Wait action failed or timed out.";
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

std::string LLMProcessorNode::adjust_camera_action(double pitch_angle, const std::string& drone_id) {
    ROS_INFO_STREAM("Dispatching to " << drone_id << ": Adjusting camera pitch to " << pitch_angle);

    nlp_drone_control::ExecuteDroneAction srv;
    srv.request.action_type = "adjust_camera_pitch"; 
    srv.request.params_json = json{{"pitch_angle", pitch_angle}}.dump();

    // 从 map 中查找并调用特定无人机的服务客户端
    if (execute_action_clients_.at(drone_id).call(srv) && srv.response.success) {
        return "Success";
    }
    
    ROS_ERROR_STREAM("Failed to execute adjust_camera_action for " << drone_id);
    return "Error: Failed to execute adjust_camera_pitch action for " + drone_id;
}

bool LLMProcessorNode::rotate_yaw_action(double degrees, const std::string& drone_id) {
    nlp_drone_control::ExecuteDroneAction srv;
    srv.request.action_type = "rotate_drone_yaw_relative";
    srv.request.params_json = json{{"angle_degrees", degrees}}.dump();
    return execute_action_clients_.at(drone_id).call(srv) && srv.response.success;
}

std::string LLMProcessorNode::executeTool(const json& tool_call, const std::string& drone_id){
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
    
    ROS_INFO_STREAM("[" << drone_id << "] Executing tool: " << tool_name << " with input: " << tool_input.dump());
    std::string result_msg = "";

    if (tool_name == "takeoff") {
        result_msg = takeoff_action(drone_id);
    } else if (tool_name == "land") {
        result_msg = land_action(drone_id);
   } else if (tool_name == "go_to_waypoint" || tool_name == "return_to_launch") {
        // 首先，发送指令
        if (tool_name == "go_to_waypoint") {
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
        } else { // 如果是 return_to_launch
            result_msg = return_to_launch_action(drone_id);
        }

        // 如果指令发送成功，现在我们就要等待它完成
        if (result_msg.rfind("Error:", 0) != 0) {
            publishFeedback("LLM Action [" + drone_id + "]: " + result_msg); // 先发布“目标已发送”的消息
            ROS_INFO("Waiting for drone %s to complete navigation...", drone_id.c_str());
            
            ros::Rate rate(10); // 每秒检查5次
            ros::Time start_time = ros::Time::now();
            const ros::Duration NAVIGATION_TIMEOUT(90.0); // 为导航设置一个90秒的超时

    // --- 阶段1: 等待无人机开始飞行 ---
    bool started_flying = false;
    while(ros::ok() && (ros::Time::now() - start_time < NAVIGATION_TIMEOUT)) {
        DroneState current_state;
        {
            std::lock_guard<std::mutex> lock(contexts_map_mutex_);
            current_state = drone_contexts_.at(drone_id).state_enum;
        }
        if (current_state == FLYING) { // 或者你的状态机里对应的 "NAVIGATING"
            started_flying = true;
            ROS_INFO("Drone %s has started navigating.", drone_id.c_str());
            break;
        }
        rate.sleep();
    }

    if (!started_flying) {
        result_msg = "Error: Timeout waiting for drone to start navigation.";
        ROS_ERROR("Drone %s: %s", drone_id.c_str(), result_msg.c_str());
    } else {
        // --- 阶段2: 等待无人机飞行结束 ---
        while(ros::ok() && (ros::Time::now() - start_time < NAVIGATION_TIMEOUT)) {
            DroneState current_state;
            {
                std::lock_guard<std::mutex> lock(contexts_map_mutex_);
                current_state = drone_contexts_.at(drone_id).state_enum;
            }
            if (current_state == HOVERING) {
                ROS_INFO("Drone %s has completed navigation and is now hovering.", drone_id.c_str());
                break; 
            }
            rate.sleep();
        }
    }

    // 检查最终是否超时
    if (ros::Time::now() - start_time >= NAVIGATION_TIMEOUT) {
        result_msg = "Error: Timeout during navigation process.";
        ROS_ERROR("Drone %s: %s", drone_id.c_str(), result_msg.c_str());
    }
    }
    }else if (tool_name == "wait") {
        if (tool_input.contains("duration_seconds")) {
            result_msg = wait_action(tool_input["duration_seconds"].get<double>(),drone_id);
        } else {
            result_msg = "Error: 'duration_seconds' is missing for wait tool.";
        }
    } else if (tool_name == "perform_visual_inspection") {
        if (tool_input.contains("target_name")) {
            std::string target = tool_input["target_name"].get<std::string>();
            result_msg = perform_visual_inspection_action(target,drone_id); // <-- Pass the target name
        } else {
            result_msg = "Error: 'target_name' parameter is missing for inspection_sequence.";
        }
    } else {
        result_msg = "Error: Unknown or unsupported tool: " + tool_name;
    }
    
    if (result_msg.rfind("Error:", 0) != 0) {
        publishFeedback("LLM Action [" + drone_id + "]: " + result_msg);
    }
    
    return result_msg;
}

bool LLMProcessorNode::execute_and_wait_for_completion(const std::string& drone_id, const std::string& action_type, const std::string& params_json, const std::string& step_name, double timeout_sec ) {
    ROS_INFO_STREAM("[" << drone_id << "] Executing step '" << step_name << "'...");

    // 1. 重置完成标志
    {
        std::lock_guard<std::mutex> map_lock(contexts_map_mutex_);
        auto& context = drone_contexts_.at(drone_id);
        std::lock_guard<std::mutex> context_lock(context.context_data_mutex);
        context.action_completed = false;
    }

    // 2. 发送动作指令
    nlp_drone_control::ExecuteDroneAction srv;
    srv.request.action_type = action_type;
    srv.request.params_json = params_json;
    if (!execute_action_clients_.at(drone_id).call(srv) || !srv.response.success) {
        ROS_ERROR_STREAM("Failed to send command for step '" << step_name << "' for " << drone_id);
        return false;
    }

    // 3. 等待 ACTION_COMPLETE 反馈
    ros::Rate rate(20);
    ros::Time start_time = ros::Time::now();
    while (ros::ok()) {
        bool completed = false;
        {
            std::lock_guard<std::mutex> map_lock(contexts_map_mutex_);
            auto& context = drone_contexts_.at(drone_id);
            std::lock_guard<std::mutex> context_lock(context.context_data_mutex);
            completed = context.action_completed;
        }
        if (completed) {
            ROS_INFO_STREAM("Step '" << step_name << "' for " << drone_id << " completed.");
            return true;
        }
        if (ros::Time::now() - start_time > ros::Duration(timeout_sec)) {
            ROS_ERROR("Timeout waiting for action completion from %s for step '%s'", drone_id.c_str(), step_name.c_str());
            return false;
        }
        ros::spinOnce(); // 处理回调
        rate.sleep();
    }
    return false; // 如果 ros::ok() 变为 false，则退出
}
std::string LLMProcessorNode::perform_visual_inspection_action(const std::string& target_name, const std::string& drone_id) {
    // 1. --- 前提条件检查 ---
    {
        std::lock_guard<std::mutex> lock(contexts_map_mutex_);
        if (drone_contexts_.at(drone_id).state_enum != HOVERING) {
            return "Error: Drone must be hovering to start inspection.";
        }
        if (_pois.find(target_name) == _pois.end()) {
            return "Error: POI '" + target_name + "' not found in the known points of interest.";
        }
    }

    publishFeedback("[" + drone_id + "] Starting inspection of '" + target_name + "'.");

    // 2. --- 稳定阶段 ---
    const double stabilization_delay = 2.0;
    publishFeedback("[" + drone_id + "] Arrived at observation point. Stabilizing for " + std::to_string(stabilization_delay) + " seconds...");
    ros::Duration(stabilization_delay).sleep();
    publishFeedback("[" + drone_id + "] Stabilization complete. Starting inspection sequence.");

    // 3. --- 计算初始转向角度 ---
    publishFeedback("[" + drone_id + "] Phase 1/3: Orienting towards target...");
    geometry_msgs::PoseStamped stable_pose;
    geometry_msgs::Point target_pos = _pois.at(target_name).position;
    {
        std::lock_guard<std::mutex> lock(contexts_map_mutex_);
        stable_pose = drone_contexts_.at(drone_id).pose;
    }
    
    // (这段计算角度的代码保持不变，因为它是正确的)
    tf2::Quaternion q_ros;
    tf2::fromMsg(stable_pose.pose.orientation, q_ros);
    tf2::Vector3 forward_vec_ros_3d = tf2::Transform(q_ros) * tf2::Vector3(1, 0, 0);
    tf2::Vector3 target_dir_ros_3d(target_pos.x - stable_pose.pose.position.x,
                                   target_pos.y - stable_pose.pose.position.y,
                                   target_pos.z - stable_pose.pose.position.z);
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
        return "Inspection failed: Target is vertically aligned, cannot orient.";
    }
    target_dir_ros_2d.normalize();
    double current_yaw_rad_ros = atan2(forward_vec_ros_2d.y(), forward_vec_ros_2d.x());
    double target_yaw_rad_ros  = atan2(target_dir_ros_2d.y(), target_dir_ros_2d.x());
    double delta_yaw_rad_ros = target_yaw_rad_ros - current_yaw_rad_ros;
    while (delta_yaw_rad_ros > M_PI)  delta_yaw_rad_ros -= 2.0 * M_PI;
    while (delta_yaw_rad_ros <= -M_PI) delta_yaw_rad_ros += 2.0 * M_PI;
    double unity_api_angle_deg = - (delta_yaw_rad_ros * 180.0 / M_PI);

    // 4. --- 执行动作序列，全部使用 execute_and_wait_for_completion ---

    // 步骤 4.1: 初始转向
    if (std::abs(unity_api_angle_deg) > 5.0) {
        std::string params = json{{"angle_degrees", unity_api_angle_deg}}.dump();
        if (!execute_and_wait_for_completion(drone_id, "rotate_drone_yaw_relative", params, "Initial Orient")) {
            return "Inspection failed at orientation step.";
        }
    } else {
        publishFeedback("[" + drone_id + "] Already facing target, skipping initial orientation.");
    }
    ros::Duration(1.0).sleep(); // 动作间短暂延迟，给物理系统稳定时间

    // 步骤 4.2: 执行后退
    publishFeedback("[" + drone_id + "] Phase 2/3: Backing up for a better view...");
    std::string backup_params = json{{"distance", 2.0}}.dump();
    if (!execute_and_wait_for_completion(drone_id, "backup", backup_params, "Backup")) {
        return "Inspection failed at backup step.";
    }
    ros::Duration(1.0).sleep();

    // 步骤 4.3: 执行拍照序列
    publishFeedback("[" + drone_id + "] Phase 3/3: Performing multi-angle visual scan...");

    // (文件和图像收集准备代码保持不变)
    std::vector<sensor_msgs::Image> collected_images;
    sensor_msgs::Image current_image;
    std::string uav_namespace = drone_id;
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
    std::string timestamp_folder = ss.str();
    std::string save_directory = image_save_base_path_ + "/" + uav_namespace + "/" + timestamp_folder;
    try {
        std::filesystem::create_directories(save_directory);
    } catch (const std::filesystem::filesystem_error& e) {
        ROS_ERROR_STREAM("Failed to create save directory: " << e.what());
        return "Error: Could not create image save directory " + save_directory;
    }
    publishFeedback("Photos will be saved to: " + save_directory);
    auto capture_photo = [&](const std::string& filename) -> bool {
        if (!capture_and_get_image(save_directory, filename, current_image, drone_id)) {
            return false;
        }
        collected_images.push_back(current_image);
        return true;
    };

    const double INSPECTION_YAW_ANGLE = 60.0;

    // --- 拍照子序列 ---
    publishFeedback("[" + drone_id + "] Step 1/5: Scanning UP/DOWN/FORWARD...");
    
    // 倾斜向上并拍照
    if (!execute_and_wait_for_completion(drone_id, "adjust_camera_pitch", json{{"pitch_angle", -45.0}}.dump(), "Tilt Up")) return "Inspection failed at Tilt Up step.";
    if (!capture_photo("0_up.jpg")) return "Inspection failed during photo capture (up).";
    
    // 倾斜向下并拍照
    if (!execute_and_wait_for_completion(drone_id, "adjust_camera_pitch", json{{"pitch_angle", 45.0}}.dump(), "Tilt Down")) return "Inspection failed at Tilt Down step.";
    if (!capture_photo("1_down.jpg")) return "Inspection failed during photo capture (down).";
    
    // 恢复水平并拍照
    if (!execute_and_wait_for_completion(drone_id, "adjust_camera_pitch", json{{"pitch_angle", 0.0}}.dump(), "Level Camera")) return "Inspection failed at Level Camera step.";
    if (!capture_photo("2_forward.jpg")) return "Inspection failed during photo capture (forward).";

    publishFeedback("[" + drone_id + "] Step 2/5: Rotating LEFT for photo...");
    // 向左转并拍照
    std::string rotate_left_params = json{{"angle_degrees", -INSPECTION_YAW_ANGLE}}.dump();
    if (!execute_and_wait_for_completion(drone_id, "rotate_drone_yaw_relative", rotate_left_params, "Rotate Left")) return "Inspection failed at Rotate Left step.";
    if (!capture_photo("3_left.jpg")) return "Inspection failed during photo capture (left).";
    
    publishFeedback("[" + drone_id + "] Step 3/5: Rotating RIGHT for photo...");
    // 向右转并拍照
    std::string rotate_right_params = json{{"angle_degrees", 2.0 * INSPECTION_YAW_ANGLE}}.dump();
    if (!execute_and_wait_for_completion(drone_id, "rotate_drone_yaw_relative", rotate_right_params, "Rotate Right")) return "Inspection failed at Rotate Right step.";
    if (!capture_photo("4_right.jpg")) return "Inspection failed during photo capture (right).";

    publishFeedback("[" + drone_id + "] Step 4/5: Returning to center after inspection...");
    // 恢复朝向并拍照
    std::string return_center_params = json{{"angle_degrees", -INSPECTION_YAW_ANGLE}}.dump();
    if (!execute_and_wait_for_completion(drone_id, "rotate_drone_yaw_relative", return_center_params, "Return to Center")) return "Inspection failed at Return to Center step.";
    
    publishFeedback("[" + drone_id + "] Step 5/5: All images captured. Requesting comprehensive description from VLM...");
    
    // 5. --- 调用 VLM 服务 ---
    vlm_service::DescribeScene srv;
    srv.request.images = collected_images;
    srv.request.prompt = "你收到了从无人机同一位置、多角度拍摄的五张照片(上、下、前、左、右)。请综合分析所有信息，提供一个关于当前场景的全面描述，重点关注设备状态或潜在异常。";
    
    if (describe_scene_client_.call(srv) && srv.response.success) {
        return "Visual Inspection Report: " + srv.response.description;
    } else {
        return "Error: Failed to get a comprehensive description from the VLM service.";
    }
}

void LLMProcessorNode::publishFeedback(const std::string& text) {
    std_msgs::String msg;
    msg.data = text;
    central_nlp_feedback_pub_.publish(msg);
}
bool LLMProcessorNode::capture_and_get_image(const std::string& dir, const std::string& filename, sensor_msgs::Image& out_image, const std::string& drone_id) {
    
    // 动态构建该无人机的摄像头话题全名
    // 这是最稳健的方式，确保我们请求的是正确的图像流
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
bool LLMProcessorNode::backup_action(double distance, const std::string& drone_id) {
    ROS_INFO_STREAM("Dispatching to " << drone_id << ": Backing up by " << distance << " meters.");
    
    nlp_drone_control::ExecuteDroneAction srv;
    srv.request.action_type = "backup";
    srv.request.params_json = json{{"distance", distance}}.dump();

    // 从 map 中查找并调用特定无人机的服务客户端
    if (execute_action_clients_.at(drone_id).call(srv) && srv.response.success) {
        return true;
    }

    ROS_ERROR_STREAM("Failed to execute backup action for " << drone_id << " for " << distance << " meters.");
    return false;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "llm_processor_node");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner; // 使用多线程Spinner，它会自动管理线程
    LLMProcessorNode node(nh);
    spinner.spin();  
    return 0;
}
