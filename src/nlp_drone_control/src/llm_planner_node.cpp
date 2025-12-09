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

  // Context from executor
  context_text_sub_        = nh_.subscribe("/fleet/context_text", 1, &LLMPlannerNode::contextTextCallback, this);
context_image_sub_ = nh_.subscribe("/fleet/context_image", 1, &LLMPlannerNode::contextImageCallback, this);

scheduler_client_ = nh_.serviceClient<uav_scheduler::ComputeAssignment>("compute_assignment");
list_pois_client_ = nh_.serviceClient<poi_state_server::ListPOIs>("/poi_state_server/list_pois");

  // Publishers (same topic names as original for feedback/reasoning)
  plan_pub_                = nh_.advertise<nlp_drone_control::Plan>("/planner/tool_calls", 10);  
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

    if (json_str.empty() || json_str == "[DONE]") {
      ROS_INFO_STREAM("[SKIP] Empty or [DONE] marker");
      continue;
    }

    try {
      if (!json::accept(json_str)) {
        ROS_WARN_STREAM("[WAIT] Incomplete JSON, keep in buffer (len=" << json_str.size() << "): " << json_str.substr(0,200) << "...");
        return true;
      }
      json response_part = json::parse(json_str);

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

        if (!content.contains("parts")) {
          ROS_WARN_STREAM("[SKIP] content without parts: " << content.dump(2));
          continue;
        }

        const auto& parts = content["parts"];

        for (const auto& part : parts) {
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
if (mode == RequestMode::TOOL_CALLS) {
  if (chosen_algo_.empty() || chosen_algo_ == "None") {
    ROS_WARN("[ACTIONS] No scheduler chosen. Attempting to execute direct plan...");

    // 1) 拿一下环境里真实存在的 POI 名集合，方便过滤显式分配中的无效条目
    std::unordered_set<std::string> env_pois;
    {
      poi_state_server::ListPOIs poi_srv;
      if (list_pois_client_.call(poi_srv) && poi_srv.response.success) {
        for (const auto& p : poi_srv.response.pois) env_pois.insert(p.name);
      } else {
        ROS_WARN("[ACTIONS] Failed to query /poi_state_server/list_pois, "
                 "will not filter user assignments by existence.");
      }
    }

    // 2) 优先解析“用户显式分配”
    auto manual = parseManualAssignments(last_user_command_);

    // 2.1 过滤掉不存在的 POI（只告警，不报错；全部无效再报错）
    if (!env_pois.empty() && !manual.empty()) {
      std::vector<std::string> missing;
      for (auto& kv : manual) {
        auto& plist = kv.second;
        plist.erase(std::remove_if(plist.begin(), plist.end(),
                      [&](const std::string& p){
                        if (env_pois.count(p)) return false;
                        missing.push_back(p);
                        return true;
                      }), plist.end());
      }
      if (!missing.empty()) {
        std::stringstream ss;
        ss << "Warning: The following POIs do not exist in the environment: ";
        for (size_t i = 0; i < missing.size(); ++i) {
          ss << missing[i] << (i + 1 < missing.size() ? ", " : "");
        }
        ROS_WARN_STREAM("[ACTIONS] " << ss.str());
        publishFeedback(ss.str());
      }
      // 如果手动分配里所有 POI 都被过滤没了，manual 可能为空或都是空列表
      bool has_any = false;
      for (const auto& kv : manual) if (!kv.second.empty()) { has_any = true; break; }
      if (!has_any) manual.clear();
    }

    // 3) 如果有“有效的显式分配”，就直接按它构建 plan
    if (!manual.empty()) {
      ROS_INFO("[ACTIONS] User-specified assignments detected. Executing them directly.");
      std::string plan_text = buildDirectPlanText(manual);
      last_plan_text_ = plan_text;            // 记下来，便于 UI 查看/缓存
      auto actions     = convertTextPlanToActions(plan_text);
      if (actions.empty()) {
        ROS_ERROR("[ACTIONS] Failed to generate actions from manual plan text.");
        publishFeedback("Error: Failed to generate actions from user-specified assignments.");
        return;
      }
      nlp_drone_control::Plan plan_msg;
      plan_msg.replan_mode = 1;
      plan_msg.actions     = actions;
      plan_pub_.publish(plan_msg);
      publishFeedback("Executed user-specified direct assignments.");
      return;
    }

    // 4) 否则，使用“简单直连”映射（用 REASONING 阶段保留下来的有效 requested_pois_）
    if (!requested_pois_.empty()) {
      auto simple_map = assignTasksByDistance(configured_drones_, requested_pois_);
      std::string plan_text = buildDirectPlanText(simple_map);
      last_plan_text_ = plan_text;
      auto actions     = convertTextPlanToActions(plan_text);
      if (actions.empty()) {
        ROS_ERROR("[ACTIONS] Failed to generate actions from simple direct plan.");
        publishFeedback("Error: No valid direct plan or assignment found.");
        return;
      }
      nlp_drone_control::Plan plan_msg;
      plan_msg.replan_mode = 1;
      plan_msg.actions     = actions;
      plan_pub_.publish(plan_msg);
      publishFeedback("Direct plan generated (Decision=None). Executing now.");
      return;
    }

    // 5) 再不行就看看有没有缓存的计划文本（理论上不会走到）
    if (!last_plan_text_.empty()) {
      auto actions = convertTextPlanToActions(last_plan_text_);
      if (!actions.empty()) {
        nlp_drone_control::Plan plan_msg;
        plan_msg.replan_mode = 1;
        plan_msg.actions     = actions;
        plan_pub_.publish(plan_msg);
        publishFeedback("Executing cached plan directly.");
        return;
      }
    }

    // 6) 三无：显式分配没有、有效 POI 没有、缓存计划也没有 -> 中断
    ROS_ERROR("[ACTIONS] No valid plan found for Decision=None. Aborting.");
    publishFeedback("Error: No valid direct plan or assignment found.");
    return;
  }
      request_body["toolConfig"] = { {"functionCallingConfig", {{"mode", "ANY"}}} };
    request_body["tools"] = { getGeminiToolDefinitions() };
    ROS_INFO("[DEBUG EXEC] Added toolConfig + tools to request_body");
}

  // System instruction
  std::string final_system_prompt;
  if (mode == RequestMode::REASONING_ONLY) {
final_system_prompt = system_prompt_content_ + "\n" + latest_context_text_ + R"(
You are a drone fleet scheduling expert with multi-modal reasoning ability.
You will receive a user instruction and a top-down map image of the environment.

Your tasks are:
1. Analyze the user instruction carefully.
2. From the map image, identify all visible POIs (buildings, houses, machines, churches, walls, etc.) and their names.
3. Compare the POIs mentioned in the instruction with those visible in the map.
   ✅ If any requested POI does not exist in the map, explicitly state that it does not exist.
   ✅ If some requested POIs exist and others do not, continue the plan ONLY with those that exist, and clearly explain this decision in the reasoning.
   ✅ If none exist, say that the instruction cannot be executed.
   Use the visual map to justify your reasoning — describe what you see in the image that supports your conclusion (e.g., "House2 is labeled in the lower-left corner, but House100 is not present anywhere").

4. Based on the valid POIs, decide which scheduling algorithm is most suitable from:
   - Hungarian Algorithm
   - Auction Algorithm
   - Genetic Algorithm
   - Distributed Auction Algorithm
   - None (if no scheduling is required)
5. Your output MUST strictly follow this format:

Reasoning:
<step-by-step reasoning here, mentioning which POIs exist or not, and why you chose the algorithm>

Decision:
<Hungarian / Auction / Genetic / Distributed Auction / None>
)";
  } else {
    final_system_prompt = "Translator mode: convert plan into tool calls only.";
  }
  request_body["systemInstruction"] = {{"parts", {{{"text", final_system_prompt}}}}};
  request_body["generationConfig"] = { {"temperature", 0.0} };

// 模型名改成 gemini-2.0-flash （最新、可用）
std::string model_name = "gemini-2.0-flash";

// 使用 streamGenerateContent 流式接口
std::string api_endpoint =
    "https://api.zhizengzeng.com/google/v1beta/models/"
    + model_name + ":streamGenerateContent";

// === 发送请求 ===
ROS_INFO("[DEBUG EXEC] Sending request via CPR...");
cpr::WriteCallback write_callback(
    std::function<bool(const std::string_view&, intptr_t)>(
        [this](const std::string_view& data, intptr_t) -> bool {
            if (interrupt_llm_request_.load()) return false;
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

  // === 检查 Reasoning 是否包含“未找到POI”等错误信息 ===
  std::string lower_reason = last_reasoning_text_;
  std::transform(lower_reason.begin(), lower_reason.end(), lower_reason.begin(), ::tolower);
// === [MODIFY] 更智能地处理 "None" 决策 ===
bool has_missing_poi =
    (lower_reason.find("does not exist") != std::string::npos ||
     lower_reason.find("not found") != std::string::npos);

if (has_missing_poi) {
    ROS_WARN("[LLMPlanner] Some POIs are missing, but will continue with valid ones.");
    publishFeedback("Warning: Some POIs mentioned do not exist. Continuing with valid ones only.");
}

if (chosen_algo_ == "None") {
    // 如果推理明确表示不需要算法（例如任务太简单或用户已指定）
    ROS_INFO("[LLMPlanner] No scheduling algorithm needed (Decision=None). Proceeding directly.");
    publishFeedback("Reasoning complete. No scheduling algorithm needed. Proceeding with direct plan.");
}

// === 从用户命令中提取 POIs，并缓存 ===
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
// === [ADD] 验证提取出的 POI 是否真实存在于环境中 ===
if (!requested_pois_.empty()) {
  poi_state_server::ListPOIs poi_srv;
  std::vector<std::string> valid_pois;
  std::vector<std::string> missing_pois;

  if (list_pois_client_.call(poi_srv) && poi_srv.response.success) {
    std::unordered_set<std::string> env_pois;
    for (const auto& poi : poi_srv.response.pois) {
      env_pois.insert(poi.name);
    }

    for (const auto& p : requested_pois_) {
      if (env_pois.count(p)) {
        valid_pois.push_back(p);
      } else {
        missing_pois.push_back(p);
      }
    }

    if (!missing_pois.empty()) {
      std::stringstream ss;
      ss << "Warning: The following POIs do not exist in the environment: ";
      for (size_t i = 0; i < missing_pois.size(); ++i) {
        ss << missing_pois[i];
        if (i + 1 < missing_pois.size()) ss << ", ";
      }
      ROS_WARN_STREAM("[LLMPlanner] " << ss.str());
      publishFeedback(ss.str());
      //chosen_algo_.clear();
      //return;  // ❌ 中断执行，不进入下一阶段
    
        // ✅ 更新有效 POI 列表
    requested_pois_ = valid_pois;
      if (requested_pois_.empty()) {
    ROS_ERROR("[LLMPlanner] All requested POIs are invalid. Aborting reasoning.");
    publishFeedback("Error: All requested POIs are invalid. Task aborted.");
    return;  // 只有全部无效才中断
  } else {
    ROS_INFO_STREAM("[LLMPlanner] Continuing with valid POIs: " << requested_pois_.size());
  }
    }


  } else {
    ROS_WARN("[LLMPlanner] Failed to query /poi_state_server/list_pois, skipping POI validation.");
  }
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

        if (scheduler_client_.call(srv)&& srv.response.assignments.empty()) {
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
  ROS_WARN("[ACTIONS] Scheduler service failed or returned invalid result. Fallback to local assignment.");
  auto local_map = assignTasksByDistance(configured_drones_, requested_pois_);
  std::string plan_text = buildDirectPlanText(local_map);
  last_plan_text_ = plan_text;
  auto actions = convertTextPlanToActions(plan_text);
  nlp_drone_control::Plan plan_msg;
  plan_msg.replan_mode = 1;
  plan_msg.actions = actions;
  plan_pub_.publish(plan_msg);
  publishFeedback("Fallback plan executed (local assignment).");
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
// ---- 解析用户显式分配： V_UAV_<id> inspect <poi> ----
static std::map<std::string, std::vector<std::string>>
parseManualAssignments(const std::string& text) {
  std::map<std::string, std::vector<std::string>> out;
  std::regex r(R"((V_UAV_\d+)\s+.*?\binspect\b\s+([A-Za-z0-9_]+))",
               std::regex::icase);
  auto it  = std::sregex_iterator(text.begin(), text.end(), r);
  auto end = std::sregex_iterator();
  for (; it != end; ++it) {
    std::string drone = it->str(1);
    std::string poi   = it->str(2);
    out[drone].push_back(poi);
  }
  return out;
}

// ---- 没有显式分配时的最简单直连规划（轮转分配） ----

static double computeDistance(const geometry_msgs::Pose& a,
                              const geometry_msgs::Pose& b) {
  return std::sqrt(std::pow(a.position.x - b.position.x, 2) +
                   std::pow(a.position.y - b.position.y, 2) +
                   std::pow(a.position.z - b.position.z, 2));
}

static std::map<std::string, std::vector<std::string>>
assignTasksByDistance(const std::vector<std::string>& drones,
                      const std::vector<std::string>& pois) {
  std::map<std::string, std::vector<std::string>> assignments;
  if (drones.empty() || pois.empty()) return assignments;

  ros::NodeHandle nh;
  tf::TransformListener tf_listener;
  std::map<std::string, geometry_msgs::Pose> drone_positions;
  std::map<std::string, geometry_msgs::Pose> poi_positions;

  // --- 获取无人机位置 ---
  for (const auto& d : drones) {
    geometry_msgs::PoseStamped pose;
    try {
      tf::StampedTransform transform;
      tf_listener.waitForTransform("world", d + "/base_link",
                                   ros::Time(0), ros::Duration(1.0));
      tf_listener.lookupTransform("world", d + "/base_link", ros::Time(0), transform);
      pose.pose.position.x = transform.getOrigin().x();
      pose.pose.position.y = transform.getOrigin().y();
      pose.pose.position.z = transform.getOrigin().z();
      drone_positions[d] = pose.pose;
    } catch (tf::TransformException& ex) {
      ROS_WARN_STREAM("[assignTasks] TF lookup failed for " << d << ": " << ex.what());
      geometry_msgs::Pose p;
      p.position.x = p.position.y = p.position.z = 0.0;
      drone_positions[d] = p;
    }
  }

  // --- 获取 POI 位置 ---
  poi_state_server::ListPOIs poi_srv;
  if (ros::service::call("/poi_state_server/list_pois", poi_srv) && poi_srv.response.success) {
    for (const auto& p : poi_srv.response.pois) {
      geometry_msgs::Pose poi_pose;
      poi_pose.position.x = p.position.x;
      poi_pose.position.y = p.position.y;
      poi_pose.position.z = p.position.z;
      poi_positions[p.name] = poi_pose;
    }
  } else {
    ROS_WARN("[assignTasks] Failed to get POI positions. Using default 0,0,0.");
    for (const auto& p : pois) {
      geometry_msgs::Pose dummy;
      dummy.position.x = dummy.position.y = dummy.position.z = 0.0;
      poi_positions[p] = dummy;
    }
  }

  // --- 距离矩阵 (drone -> poi) ---
  struct Candidate {
    std::string drone;
    std::string poi;
    double dist;
  };
  std::vector<Candidate> candidates;

  for (const auto& d : drones) {
    for (const auto& p : pois) {
      if (poi_positions.count(p))
        candidates.push_back({d, p, computeDistance(drone_positions[d], poi_positions[p])});
    }
  }

  // 按距离升序排序
  std::sort(candidates.begin(), candidates.end(),
            [](const Candidate& a, const Candidate& b) { return a.dist < b.dist; });

  std::unordered_set<std::string> assigned_drones;
  std::unordered_set<std::string> assigned_pois;

  // --- 最近匹配 ---
  for (const auto& c : candidates) {
    if (assigned_drones.count(c.drone) || assigned_pois.count(c.poi))
      continue;  // 已被分配，跳过

    assignments[c.drone].push_back(c.poi);
    assigned_drones.insert(c.drone);
    assigned_pois.insert(c.poi);

    ROS_INFO_STREAM("[assignTasks] " << c.drone << " -> " << c.poi << " (dist=" << c.dist << ")");
  }

  // ✅ 不再给空任务 UAV 生成 plan
  return assignments;
}

// ---- 按你执行器认识的格式，生成“可执行步骤文本” ----
static std::string buildDirectPlanText(
    const std::map<std::string, std::vector<std::string>>& assignments) {
  std::stringstream plan_ss;
  for (const auto& kv : assignments) {
    const auto& drone = kv.first;
    const auto& plist = kv.second;

    plan_ss << drone << ":\n";
    int step = 1;
    plan_ss << "  " << step++ << ". Take off from the current location.\n";
    if (plist.empty()) {
      continue;  // 没任务的不生成任何步骤
    } else {
      for (const auto& poi : plist) {
        plan_ss << "  " << step++ << ". Go to the best approach point for " << poi << ".\n";
        plan_ss << "  " << step++ << ". Inspect " << poi << ".\n";
      }
    }
    plan_ss << "  " << step++ << ". Return to the launch location.\n";
    plan_ss << "  " << step++ << ". Land.\n";
  }
  return plan_ss.str();
}


