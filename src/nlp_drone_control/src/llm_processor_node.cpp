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
// 在 llm_processor_node.cpp 文件中

// 【最终版本】替换你整个的 streamCallback 函数
bool LLMProcessorNode::streamCallback(const std::string& chunk) {
    std::lock_guard<std::mutex> buffer_lock(stream_buffer_mutex_);
    stream_buffer_ += chunk;

    while (true) {
        size_t end_pos = stream_buffer_.find("\n\n");
        if (end_pos == std::string::npos) break;

        std::string message_block = stream_buffer_.substr(0, end_pos);
        stream_buffer_.erase(0, end_pos + 2);

        std::stringstream ss(message_block);
        std::string line;
        while (std::getline(ss, line)) {
            if (line.rfind("data: ", 0) != 0) continue;
            
            std::string data_str = line.substr(6);
            if (data_str == "[DONE]") continue;

            try {
                json delta_json = json::parse(data_str);
                // --- 【【【 最核心的调试日志 】】】 ---
                // 打印出整个 delta_json 的内容，这样我们能看到所有字段
                ROS_INFO_STREAM("--- RECEIVED DELTA JSON ---\n" << delta_json.dump(2));
                // --- 【【【 END OF DEBUG LOG 】】】 ---
                
                if (!delta_json.contains("choices") || delta_json["choices"].empty()) continue;

                const auto& delta = delta_json["choices"][0]["delta"];

                // 1. 处理思考过程的文本流 (content)
                if (delta.contains("content") && delta["content"].is_string()) {
                    accumulated_reasoning_response_ += delta["content"].get<std::string>();
                    // 实时发布思考过程
                    size_t newline_pos;
                    while ((newline_pos = accumulated_reasoning_response_.find('\n')) != std::string::npos) {
                        std::string line_to_publish = accumulated_reasoning_response_.substr(0, newline_pos);
                        if (!line_to_publish.empty()) {
                            publishReasoning(line_to_publish);
                        }
                        accumulated_reasoning_response_.erase(0, newline_pos + 1);
                    }
                }

                // 2. 处理工具调用的结构化流 (tool_calls)
                if (delta.contains("tool_calls")) {
                    for (const auto& tool_call_chunk : delta["tool_calls"]) {
                        int index = tool_call_chunk["index"];

                        // 如果是新的工具调用，创建基本结构
                        if (accumulated_tool_calls_.size() <= index) {
                            accumulated_tool_calls_.push_back(json::object({{"index", index}}));
                        }

                        // 累积 id, type, function name
                        if (tool_call_chunk.contains("id")) {
                            accumulated_tool_calls_[index]["id"] = tool_call_chunk["id"];
                        }
                        if (tool_call_chunk.contains("type")) {
                            accumulated_tool_calls_[index]["type"] = tool_call_chunk["type"];
                        }
                        if (tool_call_chunk.contains("function")) {
                            if (tool_call_chunk["function"].contains("name")) {
                                if (!accumulated_tool_calls_[index].contains("function")) {
                                    accumulated_tool_calls_[index]["function"] = json::object();
                                }
                                accumulated_tool_calls_[index]["function"]["name"] = tool_call_chunk["function"]["name"];
                            }
                            // 累积 arguments 片段
                            if (tool_call_chunk["function"].contains("arguments")) {
                                if (!accumulated_tool_calls_[index]["function"].contains("arguments")) {
                                    accumulated_tool_calls_[index]["function"]["arguments"] = "";
                                }
                                accumulated_tool_calls_[index]["function"]["arguments"] = accumulated_tool_calls_[index]["function"]["arguments"].get<std::string>() + tool_call_chunk["function"]["arguments"].get<std::string>();
                            }
                        }
                    }
                }

            } catch (const json::exception& e) {
                ROS_WARN_STREAM("Stream parsing error: " << e.what() << " | Data: " << data_str);
            }
        }
    }
    return true;
}


// 【最终版本】替换你整个的 executeLlmRequest 函数
void LLMProcessorNode::executeLlmRequest(std::string command_to_process) {
    interrupt_llm_request_.store(false);
    publishFeedback("LLM is thinking...");

    // 重置状态
    accumulated_tool_calls_ = json::array();
    accumulated_reasoning_response_.clear();
    
    // 构建标准的OpenAI请求体
    std::string dynamic_info_str = getSwarmStateAsText();
    std::string final_system_prompt = system_prompt_content_ + "\n" + dynamic_info_str;
    
    json conversation_for_api = json::array();
    conversation_for_api.push_back({{"role", "system"}, {"content", final_system_prompt}});
    // 注意：这里可以添加更多历史记录，但为了简化，我们只用最新的
    conversation_for_api.push_back({{"role", "user"}, {"content", command_to_process}});

    json request_body = {
        {"model", "gpt-4o"},
        {"messages", conversation_for_api},
        {"tools", getFleetToolDefinitions()},
        {"tool_choice", "auto"},
        {"stream", true}
    };

    ROS_INFO_STREAM("--- Sending LLM Request ---");

    cpr::WriteCallback write_callback{[this](const std::string_view& data, intptr_t userdata) -> bool {
        if (interrupt_llm_request_.load()) return false;
        return this->streamCallback(std::string(data));
    }};

    cpr::Response r = cpr::Post(
        cpr::Url{"https://api.zhizengzeng.com/v1/chat/completions"},
        cpr::Header{{"Authorization", "Bearer " + api_key_}, {"Content-Type", "application/json"}},
        cpr::Body{request_body.dump()},
        write_callback,
        cpr::Proxies{{"https", "http://127.0.0.1:7897"}}
    );

    // --- 请求结束后的处理 ---
    if (interrupt_llm_request_.load()) {
        ROS_INFO("LLM request interrupted by user.");
        publishFeedback("LLM thought process stopped.");
        return;
    }

    if (r.status_code == 200) {
        // 发布最后剩余的思考过程文本
        if (!accumulated_reasoning_response_.empty()) {
            publishReasoning(accumulated_reasoning_response_);
        }
        // --- 【【【 最关键的调试日志 】】】 ---
        // 在做任何其他事情之前，打印出 streamCallback 组装的最终结果
        ROS_INFO_STREAM("--- Accumulated Tool Calls at End of Stream ---\n" 
                        << accumulated_tool_calls_.dump(2));
        // --- 【【【 END OF DEBUG LOG 】】】 ---
        // 现在检查累积的工具调用
        if (!accumulated_tool_calls_.empty()) {
            publishFeedback("New plan received. Updating tasks for drones...");
            
            // 在调度之前，将我们自己构建的 tool_calls 转换为 dispatchToolCalls 期望的格式
            // 我们的 dispatch 函数期望一个简单的函数对象数组
            json functions_to_dispatch = json::array();
            for(const auto& tool_call : accumulated_tool_calls_){
                if(tool_call.contains("function")){
                    functions_to_dispatch.push_back({{"function", tool_call["function"]}});
                }
            }
            
            dispatchToolCalls(functions_to_dispatch, ReplanMode::REPLACE);
        } else {
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
        llm_request_thread_ = std::thread(&LLMProcessorNode::executeLlmRequest, this, command_to_process);
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
json args = json::parse(tool_call.at("function").at("arguments").get<std::string>());
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
"description": "【首选】命令指定的无人机在其当前位置，朝向一个目标POI，执行一次完整的、可靠的视觉检查扫描序列。这是一个复合宏任务。(PREFERRED METHOD: Commands a drone to perform a full, reliable visual inspection sequence at its current location, oriented towards a target POI. This is a compound macro-task.)",
"parameters": {
"type": "object",
"properties": {
"drone_id": { "type": "string", "description": "要执行此动作的无人机ID。" },
"target_name": { 
    "type": "string", 
    "description": "要检查的兴趣点（POI）的**精确名称**。**必须使用 'target_name' 这个键名**，例如：'House1'。(The **exact name** of the POI to inspect. **You MUST use the key name 'target_name'**, e.g., 'House1')." 
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
},
{
"type": "function",
"function": {
"name": "cancel_all_tasks",
"description": "【紧急】立即取消指定无人机当前和所有排队的任务，使其悬停等待新指令。(EMERGENCY: Immediately cancels the current and all queued tasks for a specific drone, causing it to hover.)",
"parameters": {
"type": "object",
"properties": { "drone_id": { "type": "string", "description": "要取消其任务的无人机ID。" }},
"required": ["drone_id"]
}
}
},
{
"type": "function",
"function": {
"name": "rotate_drone_yaw_relative",
"description": "【专家模式】命令无人机相对当前朝向旋转指定的偏航角度。(EXPERT MODE: Commands a drone to rotate its yaw by a specified number of degrees relative to its current heading.)",
"parameters": {
"type": "object",
"properties": {
"drone_id": { "type": "string", "description": "要执行此动作的无人机ID。" },
"angle_degrees": { "type": "number", "description": "相对旋转的角度（度）。正值为顺时针，负值为逆时针。(The relative angle to rotate in degrees. Positive is clockwise, negative is counter-clockwise.)" }
},
"required": ["drone_id", "angle_degrees"]
}
}
},
{
"type": "function",
"function": {
"name": "backup",
"description": "【专家模式】命令无人机从当前位置向后移动指定的距离。(EXPERT MODE: Commands a drone to move backward from its current position by a specified distance.)",
"parameters": {
"type": "object",
"properties": {
"drone_id": { "type": "string", "description": "要执行此动作的无人机ID。" },
"distance": { "type": "number", "description": "向后移动的距离（米）。" }
},
"required": ["drone_id", "distance"]
}
}
},
{
"type": "function",
"function": {
"name": "adjust_camera_pitch",
"description": "【专家模式】调整无人机摄像头的俯仰角。(EXPERT MODE: Adjusts the drone's camera pitch angle.)",
"parameters": {
"type": "object",
"properties": {
"drone_id": { "type": "string", "description": "要执行此动作的无人机ID。" },
"pitch_angle": { "type": "number", "description": "目标俯仰角（度）。负值为上仰，正值为下俯。(Target pitch angle in degrees. Negative values tilt up, positive values tilt down.)" }
},
"required": ["drone_id", "pitch_angle"]
}
}
}
])END");
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
send_simple_command(drone_id, "wait");
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

int main(int argc, char** argv) {
ros::init(argc, argv, "llm_processor_node");
ros::NodeHandle nh;
ros::MultiThreadedSpinner spinner; // 使用多线程Spinner，它会自动管理线程
LLMProcessorNode node(nh);
spinner.spin();
return 0;
}
