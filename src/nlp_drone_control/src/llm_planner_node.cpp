#include "nlp_drone_control/llm_planner_node.h"
#include <fstream>
#include <sstream>
#include <regex>
#include <XmlRpcValue.h> // not strictly needed on planner, but harmless

// ================== Constructor / Destructor ==================

LLMPlannerNode::LLMPlannerNode(ros::NodeHandle& nh) : nh_(nh) {
  ros::NodeHandle pnh("~");
  ROS_INFO("LLM Planner Node is initializing...");

  // Load API key & system prompt (same as your original)
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

  // Conversation history init (same as original)
  conversation_history_ = json::array();
  conversation_history_.push_back({{"role", "system"}, {"content", ""}});

  // Subscribers (same topic names as original)
  central_nlp_command_sub_ = nh_.subscribe("/nlp_command", 1, &LLMPlannerNode::centralNlpCommandCallback, this);
  rereasoning_sub_         = nh_.subscribe("/nlp_rereasoning", 1, &LLMPlannerNode::rereasoningCallback, this);
  run_sub_                 = nh_.subscribe("/nlp_run", 1, &LLMPlannerNode::runCallback, this);
  stop_command_sub_        = nh_.subscribe("/nlp_stop", 1, &LLMPlannerNode::stopCommandCallback, this);

  // [ADDED] Context from executor
  context_text_sub_        = nh_.subscribe("/fleet/context_text", 1, &LLMPlannerNode::contextTextCallback, this);
context_image_sub_ = nh_.subscribe("/fleet/context_image", 1, &LLMPlannerNode::contextImageCallback, this);

scheduler_client_ = nh_.serviceClient<uav_scheduler::ComputeAssignment>("compute_assignment");
list_pois_client_ = nh_.serviceClient<poi_state_server::ListPOIs>("/poi_state_server/list_pois");

  // Publishers (same topic names as original for feedback/reasoning)
  plan_pub_                = nh_.advertise<nlp_drone_control::Plan>("/planner/tool_calls", 10);  // [ADDED]
  central_nlp_feedback_pub_= nh_.advertise<std_msgs::String>("/nlp_feedback", 10);
  reasoning_pub_           = nh_.advertise<std_msgs::String>("/nlp_reasoning", 10);

  // Optional: keep processing thread (your original loop)
  processing_thread_ = std::thread(&LLMPlannerNode::processingLoop, this);
  
if (!nh_.getParam("/drone_ids", configured_drones_)) {
    ROS_ERROR("Parameter '/drone_ids' not found! Please set in launch file.");
} else {
    ROS_INFO_STREAM("Loaded drone_ids: " << configured_drones_.size());
    for (const auto& id : configured_drones_) {
        ROS_INFO_STREAM(" - " << id);
    }
}

  ROS_INFO("LLM Planner Node is ready.");
}

LLMPlannerNode::~LLMPlannerNode() {
  if (processing_thread_.joinable()) {
    processing_thread_.join();
  }
  std::lock_guard<std::mutex> lock(llm_thread_mutex_);
  if (llm_request_thread_.joinable()) {
    llm_request_thread_.join();
  }
}
std::string encodeImageToBase64(const sensor_msgs::Image& ros_image) {
  try {
    // 拷贝/共享成 OpenCV 图像
    cv_bridge::CvImageConstPtr cv_ptr =
        cv_bridge::toCvShare(boost::make_shared<sensor_msgs::Image>(ros_image));

    // 编码为 PNG
    std::vector<uchar> buf;
    if (!cv::imencode(".png", cv_ptr->image, buf)) {
      ROS_ERROR("cv::imencode(.png) failed.");
      return "";
    }

    using namespace boost::archive::iterators;
    typedef base64_from_binary<transform_width<const unsigned char*, 6, 8>> Base64Encoder;

    std::stringstream ss;
    std::copy(Base64Encoder(buf.data()),
              Base64Encoder(buf.data() + buf.size()),
              std::ostream_iterator<char>(ss));

    // Base64 padding
    size_t padding = (3 - buf.size() % 3) % 3;
    ss << std::string(padding, '=');

    return ss.str();
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("encodeImageToBase64 failed: " << e.what());
    return "";
  }
}
// ================== User/UI Callbacks (unchanged names) ==================

void LLMPlannerNode::centralNlpCommandCallback(const std_msgs::String::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(command_mutex_);
  latest_user_command_ = msg->data;
  needs_replan_ = true;
  last_replan_reason_ = "New user command received: '" + msg->data + "'";
  ROS_INFO_STREAM("Replanning triggered by new user command.");
}

void LLMPlannerNode::stopCommandCallback(const std_msgs::Empty::ConstPtr& /*msg*/) {
  ROS_WARN("LLM STOP command received! Interrupting current thought process.");
  publishFeedback("User interrupted. LLM is stopping...");

  interrupt_llm_request_.store(true);

  std::lock_guard<std::mutex> lock(command_mutex_);
  needs_replan_ = false;
  latest_user_command_.clear();
  last_replan_reason_.clear();
}

void LLMPlannerNode::rereasoningCallback(const std_msgs::Empty::ConstPtr& /*msg*/) {
  ROS_INFO("Rereasoning command received from UI.");
  publishFeedback("User requested re-reasoning. Restarting reasoning phase...");

  interrupt_llm_request_.store(true);

  std::lock_guard<std::mutex> lock(llm_thread_mutex_);
  if (llm_request_thread_.joinable()) {
    llm_request_thread_.join();
  }
  interrupt_llm_request_.store(false);

  llm_request_thread_ = std::thread(
      &LLMPlannerNode::executeLlmRequest,
      this,
      last_user_command_,
      RequestMode::REASONING_ONLY
  );
}

void LLMPlannerNode::runCallback(const std_msgs::Empty::ConstPtr& /*msg*/) {
  //ROS_INFO("Run command received from UI.");
  publishFeedback("User confirmed execution. Running tool calls...");

  interrupt_llm_request_.store(true);

  std::lock_guard<std::mutex> lock(llm_thread_mutex_);
  if (llm_request_thread_.joinable()) {
    llm_request_thread_.join();
  }
  interrupt_llm_request_.store(false);

  llm_request_thread_ = std::thread(
      &LLMPlannerNode::executeLlmRequest,
      this,
      last_user_command_,
      RequestMode::TOOL_CALLS
  );
}

void LLMPlannerNode::contextTextCallback(const std_msgs::String::ConstPtr& msg) {
  latest_context_text_ = msg->data;
}
void LLMPlannerNode::contextImageCallback(const sensor_msgs::Image::ConstPtr& msg) {
  try {
    latest_context_image_ = *msg;  // 保存一份
    //ROS_INFO("[LLMPlanner] Received /fleet/context_image (%ux%u)",
    //         msg->width, msg->height);
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Failed to receive context image: " << e.what());
  }
}


// ================== SSE stream handling (moved verbatim) ==================

bool LLMPlannerNode::streamCallbackForGemini(const std::string& chunk) {
  std::lock_guard<std::mutex> buffer_lock(stream_buffer_mutex_);
  stream_buffer_ += chunk;

  std::istringstream ss(stream_buffer_);
  std::string line;
  size_t processed_chars = 0;

  while (std::getline(ss, line)) {
    processed_chars += line.size() + 1;

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
   // ROS_INFO_STREAM("[SSE raw line] " << json_str);
    //ROS_INFO_STREAM("[BUFFER SIZE after append] " << stream_buffer_.size());

    if (json_str.empty() || json_str == "[DONE]") {
      ROS_INFO_STREAM("[SKIP] Empty or [DONE] marker");
      continue;
    }

    try {
      if (!json::accept(json_str)) {
        ROS_WARN_STREAM("[WAIT] Incomplete JSON, keep in buffer (len=" << json_str.size() << "): " << json_str.substr(0,200) << "...");
        return true;
      }
      //ROS_INFO_STREAM("[BUFFER before parse] size=" << stream_buffer_.size());

      json response_part = json::parse(json_str);
      //ROS_INFO_STREAM("[DEBUG response_part] " << response_part.dump(2));

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
        //ROS_INFO_STREAM("[DEBUG CANDIDATE CONTENT] " << content.dump(2));

        if (!content.contains("parts")) {
          ROS_WARN_STREAM("[SKIP] content without parts: " << content.dump(2));
          continue;
        }

        const auto& parts = content["parts"];
        //ROS_INFO_STREAM("[DEBUG parts count] " << parts.size());

        for (const auto& part : parts) {
          //ROS_INFO_STREAM("[DEBUG PART RAW] " << part.dump(2));

          if (current_mode == RequestMode::REASONING_ONLY) {
            if (part.contains("text")) {
              std::string text_chunk = part["text"];
              accumulated_reasoning_response_ += text_chunk;
              publishReasoning(text_chunk);
              //ROS_INFO_STREAM("[REASONING] appended text chunk");
            } else {
              ROS_WARN_STREAM("[REASONING] part without text: " << part.dump(2));
            }
          } else if (current_mode == RequestMode::TOOL_CALLS) {
            //ROS_INFO_STREAM("Enter TOOL_CALLS mode");

            if (part.contains("functionCall") || part.contains("function_call")) {
              //ROS_INFO_STREAM("[FOUND FUNCTIONCALL] " << part.dump(2));
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
                //ROS_INFO_STREAM("[PUSHED tool call] " << formatted_tool_call.dump(2));
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
  stream_buffer_.erase(0, processed_chars);
  return true;
}

// ================== Execute LLM request (planner role) ==================

static std::string trim_local(const std::string& s) {
  auto a = s.find_first_not_of(" \t\r\n");
  if (a == std::string::npos) return "";
  auto b = s.find_last_not_of(" \t\r\n");
  return s.substr(a, b - a + 1);
}

std::string LLMPlannerNode::extractPlanFromReasoning(const std::string& md) {
  std::regex re(R"(Full Action Sequence Plan\s*\(Text Version\)\s*:\s*([\s\S]*?)(?:\n---|\n\*\*2\. Summary|\Z))");
  std::smatch m;
  if (std::regex_search(md, m, re) && m.size() > 1) {
    return trim_local(m[1].str());
  }
  // fallback: return everything after "Decision:"
  size_t pos = md.find("Decision:");
  if (pos != std::string::npos) {
    return trim_local(md.substr(pos + 9));
  }
  return trim_local(md);
}


void LLMPlannerNode::executeLlmRequest(std::string command_to_process, RequestMode mode) {
  interrupt_llm_request_.store(false);
  publishFeedback("LLM is thinking...");

  current_mode = mode;
  last_user_command_ = command_to_process;

  {
    std::lock_guard<std::mutex> lock(stream_buffer_mutex_);
    accumulated_tool_calls_ = json::array();
    accumulated_reasoning_response_.clear();
  }
  in_progress_tool_calls_.clear();

  // Build contents
  json gemini_contents = json::array();

  if (mode == RequestMode::REASONING_ONLY) {
    json user_parts = json::array();
    json text_part;
    text_part["text"] = command_to_process;
    user_parts.push_back(text_part);
    ROS_INFO_STREAM("[DEBUG EXEC] Added user command text: " << command_to_process);

    if (!latest_context_image_.data.empty()) {
      std::string image_b64 = encodeImageToBase64(latest_context_image_);
      //ROS_INFO_STREAM("[DEBUG EXEC] Context image size: " << latest_context_image_.data.size());
      if (!image_b64.empty()) {
        json img_part;
        img_part["inline_data"] = {
          {"mime_type", "image/png"},
          {"data", image_b64}
        };
        user_parts.push_back(img_part);
        //ROS_INFO("[DEBUG EXEC] Added context image to request.");
      } else {
        ROS_WARN("[DEBUG EXEC] Failed to encode context image, sending text only.");
      }
    } else {
      ROS_INFO("[DEBUG EXEC] No context image available.");
    }

    gemini_contents.push_back({{"role", "user"}, {"parts", user_parts}});
  } 
  json request_body;
  request_body["contents"] = gemini_contents;
if (mode == RequestMode::TOOL_CALLS && chosen_algo_.empty()) {
  ROS_ERROR("[ACTIONS] No chosen algorithm found! Please run reasoning first.");
  publishFeedback("Error: No chosen algorithm. Please run reasoning first.");
  return;
}

  if (mode == RequestMode::TOOL_CALLS) {
    request_body["toolConfig"] = { {"functionCallingConfig", {{"mode", "ANY"}}} };
    request_body["tools"] = { getGeminiToolDefinitions() };
    ROS_INFO("[DEBUG EXEC] Added toolConfig + tools to request_body");
  }

  // System instruction
  std::string final_system_prompt;
  if (mode == RequestMode::REASONING_ONLY) {
final_system_prompt = system_prompt_content_ + "\n" + latest_context_text_ + R"(
You are a drone fleet scheduling expert.

When given a user instruction:
1. Always analyze which scheduling algorithm is needed.
2. Explicitly choose ONE scheduling algorithm from the following list:
   - Hungarian Algorithm
   - Auction Algorithm
   - Genetic Algorithm
   - Distributed Auction Algorithm
   - None (if tasks are fully pre-assigned, explain why)
3. Your output MUST strictly follow this format:

Reasoning:
<step-by-step reasoning here>

Decision:
<Hungarian / Auction / Genetic / Distributed Auction / None>

)";
  } else {
    final_system_prompt = "Translator mode: convert plan into tool calls only.";
  }
  request_body["systemInstruction"] = {{"parts", {{{"text", final_system_prompt}}}}};
  request_body["generationConfig"] = { {"temperature", 0.0} };

  //ROS_INFO_STREAM("[DEBUG EXEC] Final request JSON size=" << request_body.dump().size());

// 模型名改成 gemini-2.0-flash （最新、可用）
std::string model_name = "gemini-2.0-flash";

// 使用 streamGenerateContent 流式接口
std::string api_endpoint =
    "https://api.zhizengzeng.com/google/v1beta/models/"
    + model_name + ":streamGenerateContent";

//ROS_INFO_STREAM("[DEBUG EXEC] API endpoint: " << api_endpoint);

// === 发送请求 ===
ROS_INFO("[DEBUG EXEC] Sending request via CPR...");
cpr::WriteCallback write_callback(
    std::function<bool(const std::string_view&, intptr_t)>(
        [this](const std::string_view& data, intptr_t) -> bool {
            if (interrupt_llm_request_.load()) return false;
            //ROS_INFO_STREAM("[DEBUG SSE CHUNK] " << data);
            return this->streamCallbackForGemini(std::string(data));
        }
    )
);

cpr::Response r = cpr::Post(
    cpr::Url{api_endpoint},
    cpr::Header{{"Content-Type", "application/json"}, {"X-goog-api-key", api_key_}},
    cpr::Body{request_body.dump()},
    write_callback,
    cpr::Proxies{{"https", "http://127.0.0.1:7897"}}
);
  ROS_INFO_STREAM("[DEBUG EXEC] Request finished. Status=" << r.status_code);

if (!r.text.empty()) {
  ROS_INFO_STREAM("[DEBUG EXEC] Response text (first 300 chars): "
                  << r.text.substr(0, 300));
} else {
  ROS_INFO("[DEBUG EXEC] Response text is empty (expected for SSE streaming).");
}

  if (interrupt_llm_request_.load()) {
    ROS_WARN("[DEBUG EXEC] Request interrupted by user.");
    return;
  }

  if (r.status_code == 200) {
    ROS_INFO("[DEBUG EXEC] Status 200, proceeding with response handling...");
if (current_mode == RequestMode::REASONING_ONLY) {
  last_reasoning_text_ = accumulated_reasoning_response_;

  // 1) 提取算法决策
chosen_algo_ = extractDecisionFromReasoning(last_reasoning_text_);
// === [ADD] 从用户命令中提取 POIs，并缓存 ===
requested_pois_.clear();
std::regex poi_regex(R"(House\d+|Church|Factory|Tower|Building\d+)",
                     std::regex::icase);  // 使用 C++ 原生大小写不敏感 flag

std::sort(requested_pois_.begin(), requested_pois_.end());
requested_pois_.erase(std::unique(requested_pois_.begin(), requested_pois_.end()), requested_pois_.end());
auto begin = std::sregex_iterator(last_user_command_.begin(), last_user_command_.end(), poi_regex);
auto end = std::sregex_iterator();
for (auto it = begin; it != end; ++it) {
  std::string poi_name = it->str();
  requested_pois_.push_back(poi_name);
}

publishFeedback("Reasoning complete. Selected algorithm: " + chosen_algo_ +
                " | Extracted POIs: " + std::to_string(requested_pois_.size()));
  
}
 else {
   // === ACTIONS ===
  ROS_INFO_STREAM("[DEBUG EXEC] Enter ACTIONS phase with decision: " << chosen_algo_);

      if (chosen_algo_ == "Hungarian Algorithm") {
        ROS_INFO("[ACTIONS] Using Hungarian scheduler...");
        uav_scheduler::ComputeAssignment srv;

        srv.request.drones = configured_drones_;
        if (srv.request.drones.empty()) {
          ROS_ERROR("[ACTIONS] No drones configured! Check /drone_ids param.");
        }

        // POI 列表：直接从用户请求 or fallback
        if (!requested_pois_.empty()) {
          srv.request.pois = requested_pois_;
        } else {
          poi_state_server::ListPOIs poi_srv;
          if (list_pois_client_.call(poi_srv) && poi_srv.response.success) {
            for (const auto& poi : poi_srv.response.pois) {
              srv.request.pois.push_back(poi.name);
            }
          }
        }

        srv.request.algorithm = chosen_algo_;

        if (scheduler_client_.call(srv)) {
          ROS_INFO("[ACTIONS] Scheduler assignment done.");
          for (const auto& a : srv.response.assignments) {
            ROS_INFO_STREAM("[ASSIGNMENT] " << a);
          }

          last_plan_text_ = srv.response.plan_text;
          has_cached_plan_.store(true);

          // ✅ 在这里解析 POIs
          requested_pois_ = extractPOIsFromPlan(last_plan_text_);
          ROS_INFO_STREAM("[DEBUG EXEC] Extracted " << requested_pois_.size() << " POIs from plan.");
          for (const auto& poi : requested_pois_) ROS_INFO_STREAM("  -> " << poi);

          auto actions = convertTextPlanToActions(last_plan_text_);
          nlp_drone_control::Plan plan_msg;
          plan_msg.replan_mode = 1;
          plan_msg.actions = actions;
          plan_pub_.publish(plan_msg);
    }
    else
    {
      ROS_ERROR("[ACTIONS] Scheduler service call failed.");
  publishFeedback("Scheduler service failed. Cannot generate plan.");
  return;
    }
}

 else if (chosen_algo_ == "Auction Algorithm") {
    ROS_INFO("[ACTIONS] Using Auction scheduler...");
    // 调用 auction_scheduler tool
  } else if (chosen_algo_ == "Genetic Algorithm") {
    ROS_INFO("[ACTIONS] Using Genetic scheduler...");
    // 调用 genetic_scheduler tool
  } else if (chosen_algo_ == "Distributed Auction Algorithm") {
    ROS_INFO("[ACTIONS] Using Distributed Auction scheduler...");
    // 调用 distributed_auction_scheduler tool
  } else {
  ROS_WARN("[ACTIONS] No scheduler chosen, executing cached plan directly.");
  auto actions = convertTextPlanToActions(last_plan_text_);
  nlp_drone_control::Plan plan_msg;
  plan_msg.actions = actions;
  plan_pub_.publish(plan_msg);
}
      if (!accumulated_tool_calls_.empty()) {
        ROS_INFO_STREAM("[DEBUG EXEC] Got " << accumulated_tool_calls_.size() << " tool calls.");
        publishFeedback("Actions complete, publishing plan...");
      }
    }
  } else {
    ROS_ERROR_STREAM("[DEBUG EXEC] LLM API request failed. Code=" << r.status_code
                     << " Body=" << r.text);
  }
}

std::string LLMPlannerNode::extractDecisionFromReasoning(const std::string& md) {
  std::regex re(R"(Decision:\s*(Hungarian Algorithm|Auction Algorithm|Genetic Algorithm|Distributed Auction Algorithm|None))");
  std::smatch m;
  if (std::regex_search(md, m, re) && m.size() > 1) {
    return trim_local(m[1].str());
  }
  return "None";
}

// ================== processingLoop (kept as-is, planner side) ==================
void LLMPlannerNode::processingLoop() {
  ros::Rate rate(5);
  while (ros::ok()) {
    std::string command_to_process;
    bool should_replan = false;

    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (needs_replan_) {
        should_replan = true;

        command_to_process = "Re-evaluate the current situation.";
        if (!last_replan_reason_.empty()) {
          command_to_process += " Triggering event: " + last_replan_reason_;
        }
        if (!latest_user_command_.empty()) {
          command_to_process += "\nNew user instruction: '" + latest_user_command_ + "'";
        }

        needs_replan_ = false;
        latest_user_command_.clear();
        last_replan_reason_.clear();
      }
    }

    if (should_replan) {
      std::lock_guard<std::mutex> lock(llm_thread_mutex_);
      if (llm_request_thread_.joinable()) {
        llm_request_thread_.join();
      }
      ROS_INFO("Starting new LLM request in a background thread (planner).");
      llm_request_thread_ = std::thread(&LLMPlannerNode::executeLlmRequest, this, command_to_process, RequestMode::REASONING_ONLY);
    }

    rate.sleep();
  }

  std::lock_guard<std::mutex> lock(llm_thread_mutex_);
  if (llm_request_thread_.joinable()) {
    llm_request_thread_.join();
  }
}

// ================== Utils kept unchanged ==================
void LLMPlannerNode::trimConversationHistory() {
  const size_t max_messages = 1 + MAX_CONVERSATION_TURNS * 2;
  if (conversation_history_.size() > max_messages) {
    conversation_history_.erase(
        conversation_history_.begin() + 1,
        conversation_history_.begin() + 1 + (conversation_history_.size() - max_messages)
    );
    ROS_INFO("Trimmed conversation history to the last %zu turns.", MAX_CONVERSATION_TURNS);
  }
}

json LLMPlannerNode::getGeminiToolDefinitions() {
    json declarations = json::array();

    // 1. Hungarian Algorithm
    declarations.push_back({
        {"name", "hungarian_scheduler"},
        {"description", "Hungarian assignment algorithm. Guarantees optimal matching between drones and POIs. Best for small to medium fleets with static tasks. Weakness: less scalable to very large fleets."},
        {"parameters", {
            {"type", "object"},
            {"properties", {
                {"drones", {{"type", "array"}, {"description", "List of drone IDs, e.g., ['V_UAV_0','V_UAV_1']"}}},
                {"pois",   {{"type", "array"}, {"description", "List of POI names, e.g., ['House1','Church']"}}}
            }},
            {"required", json::array({"drones", "pois"})}
        }}
    });

    // 2. Auction Algorithm
    declarations.push_back({
        {"name", "auction_scheduler"},
        {"description", "Auction-based task allocation. Drones bid for tasks. Lightweight and distributed. Good for dynamic or online assignment. Weakness: not guaranteed optimal."},
        {"parameters", {
            {"type", "object"},
            {"properties", {
                {"drones", {{"type", "array"}, {"description", "List of drone IDs"}}},
                {"pois",   {{"type", "array"}, {"description", "List of POIs"}}}
            }},
            {"required", json::array({"drones", "pois"})}
        }}
    });

    // 3. Genetic Algorithm
    declarations.push_back({
        {"name", "genetic_scheduler"},
        {"description", "Genetic algorithm for global optimization of task assignment. Handles large and complex problems. Weakness: slower, may waste compute."},
        {"parameters", {
            {"type", "object"},
            {"properties", {
                {"drones", {{"type", "array"}, {"description", "List of drone IDs"}}},
                {"pois",   {{"type", "array"}, {"description", "List of POIs"}}}
            }},
            {"required", json::array({"drones", "pois"})}
        }}
    });

    // 4. Distributed Auction Algorithm
    declarations.push_back({
        {"name", "distributed_auction_scheduler"},
        {"description", "Distributed version of auction. Scalable, robust against communication failures, suitable for decentralized fleets. Weakness: may converge slowly."},
        {"parameters", {
            {"type", "object"},
            {"properties", {
                {"drones", {{"type", "array"}, {"description", "List of drone IDs"}}},
                {"pois",   {{"type", "array"}, {"description", "List of POIs"}}}
            }},
            {"required", json::array({"drones", "pois"})}
        }}
    });

    return {{"function_declarations", declarations}};
}


void LLMPlannerNode::publishFeedback(const std::string& text) {
  std_msgs::String msg; msg.data = text;
  central_nlp_feedback_pub_.publish(msg);
}
void LLMPlannerNode::publishReasoning(const std::string& text) {
  std_msgs::String msg; msg.data = text;
  reasoning_pub_.publish(msg);
}
std::vector<nlp_drone_control::Action>
LLMPlannerNode::convertTextPlanToActions(const std::string& plan_text) {
  std::vector<nlp_drone_control::Action> actions;

  std::stringstream ss(plan_text);
  std::string line;
  std::string current_drone;

  bool takeoff_added = false;  // 避免重复 takeoff

  while (std::getline(ss, line)) {
    // 去掉前后空格
    line.erase(0, line.find_first_not_of(" \t"));
    line.erase(line.find_last_not_of(" \t\r\n") + 1);

    // === 检测新的 UAV ===
    if (line.find("V_UAV_") == 0 || line.find("UAV_") == 0) {
      current_drone = line.substr(0, line.find(":"));
      takeoff_added = false; // 新 UAV 重新开始
      continue;
    }

    if (current_drone.empty()) continue;  // 没有 UAV，跳过

    // === Takeoff（每个 UAV 只加一次） ===
    if (line.find("Take off") != std::string::npos && !takeoff_added) {
      nlp_drone_control::Action act;
      act.function_name = "takeoff";
      json args;
      args["drone_id"] = current_drone;
      args["altitude"] = 3.0;  // 固定高度，可调
      act.arguments_json = args.dump();
      actions.push_back(act);
      takeoff_added = true;
    }

    // === Go to + Inspect ===
    if (line.find("Go to the best approach point for") != std::string::npos) {
      // 提取 POI 名字
      size_t pos = line.find("for ");
      std::string poi = (pos != std::string::npos) ? line.substr(pos + 4) : "UNKNOWN";
  // ✅ 去掉末尾非字母数字字符
  while (!poi.empty() && !std::isalnum(poi.back())) poi.pop_back();
  poi.erase(0, poi.find_first_not_of(" \t\r\n"));
  poi.erase(poi.find_last_not_of(" \t\r\n") + 1);
      // go_to_waypoint
      {
        nlp_drone_control::Action act;
        act.function_name = "go_to_waypoint";
        json args;
        args["drone_id"] = current_drone;
        args["target_name"] = poi;
        args["z"] = 2.0; // 默认高度
        act.arguments_json = args.dump();
        actions.push_back(act);
      }

      // perform_visual_inspection
      {
        nlp_drone_control::Action act;
        act.function_name = "perform_visual_inspection";
        json args;
        args["drone_id"] = current_drone;
        args["target_name"] = poi;
        act.arguments_json = args.dump();
        actions.push_back(act);
      }
    }

    // === Return to launch ===
    if (line.find("Return to the launch location") != std::string::npos) {
      nlp_drone_control::Action act;
      act.function_name = "return_to_launch";
      json args;
      args["drone_id"] = current_drone;
      act.arguments_json = args.dump();
      actions.push_back(act);
    }

    // === Land ===
    if (line.find("Land") != std::string::npos) {
      nlp_drone_control::Action act;
      act.function_name = "land";
      json args;
      args["drone_id"] = current_drone;
      act.arguments_json = args.dump();
      actions.push_back(act);
    }
  }

  return actions;
}
std::vector<std::string> LLMPlannerNode::extractPOIsFromPlan(const std::string& plan_text) {
  std::vector<std::string> pois;
  std::stringstream ss(plan_text);
  std::string line;

  while (std::getline(ss, line)) {
    // 去掉两端空格
    line.erase(0, line.find_first_not_of(" \t\r\n"));
    line.erase(line.find_last_not_of(" \t\r\n") + 1);

    // 匹配 "Inspect ..."
    if (line.find("Inspect ") != std::string::npos) {
      size_t pos = line.find("Inspect ");
      std::string poi = line.substr(pos + 8);

      // ✅ 去除末尾所有非字母数字字符
      while (!poi.empty() && !std::isalnum(poi.back())) poi.pop_back();

      // 去除首尾空格
      poi.erase(0, poi.find_first_not_of(" \t\r\n"));
      poi.erase(poi.find_last_not_of(" \t\r\n") + 1);

      pois.push_back(poi);
    }

    // 匹配 "Go to the best approach point for ..."
    if (line.find("Go to the best approach point for ") != std::string::npos) {
      size_t pos = line.find("for ");
      std::string poi = line.substr(pos + 4);

      // ✅ 同样去除末尾非字母数字字符
      while (!poi.empty() && !std::isalnum(poi.back())) poi.pop_back();

      poi.erase(0, poi.find_first_not_of(" \t\r\n"));
      poi.erase(poi.find_last_not_of(" \t\r\n") + 1);

      pois.push_back(poi);
    }
  }

  // 去重
  std::sort(pois.begin(), pois.end());
  pois.erase(std::unique(pois.begin(), pois.end()), pois.end());

  return pois;
}

