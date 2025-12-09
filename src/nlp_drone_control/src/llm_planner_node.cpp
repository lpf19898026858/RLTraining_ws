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


  // Publishers (same topic names as original for feedback/reasoning)
  plan_pub_                = nh_.advertise<nlp_drone_control::Plan>("/planner/tool_calls", 10);  // [ADDED]
  central_nlp_feedback_pub_= nh_.advertise<std_msgs::String>("/nlp_feedback", 10);
  reasoning_pub_           = nh_.advertise<std_msgs::String>("/nlp_reasoning", 10);

  // Optional: keep processing thread (your original loop)
  processing_thread_ = std::thread(&LLMPlannerNode::processingLoop, this);

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
  ROS_INFO("Run command received from UI.");
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
  ROS_INFO_STREAM("streamCallbackForGemini current_mode = "
      << (current_mode == RequestMode::TOOL_CALLS ? "TOOL_CALLS" : "REASONING_ONLY"));

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
    ROS_INFO_STREAM("[SSE raw line] " << json_str);
    ROS_INFO_STREAM("[BUFFER SIZE after append] " << stream_buffer_.size());

    if (json_str.empty() || json_str == "[DONE]") {
      ROS_INFO_STREAM("[SKIP] Empty or [DONE] marker");
      continue;
    }

    try {
      if (!json::accept(json_str)) {
        ROS_WARN_STREAM("[WAIT] Incomplete JSON, keep in buffer (len=" << json_str.size() << "): " << json_str.substr(0,200) << "...");
        return true;
      }
      ROS_INFO_STREAM("[BUFFER before parse] size=" << stream_buffer_.size());

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
              ROS_INFO_STREAM("[REASONING] appended text chunk");
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

  if (current_mode == RequestMode::TOOL_CALLS) {
    ROS_INFO_STREAM("[SUMMARY] Accumulated tool calls so far: " << accumulated_tool_calls_.size());
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
  return trim_local(md);
}

void LLMPlannerNode::executeLlmRequest(std::string command_to_process, RequestMode mode) {
  interrupt_llm_request_.store(false);
  publishFeedback("LLM is thinking...");
  ROS_INFO_STREAM("executeLlmRequest setting current_mode = "
      << (mode == RequestMode::TOOL_CALLS ? "TOOL_CALLS" : "REASONING_ONLY"));
  current_mode = mode;

  last_user_command_ = command_to_process;
  accumulated_tool_calls_ = json::array();
  in_progress_tool_calls_.clear();
  accumulated_reasoning_response_.clear();

  // Build contents
  json gemini_contents = json::array();

if (mode == RequestMode::REASONING_ONLY) {
  json user_parts = json::array();

  // 1) 文字部分
  json text_part;
  text_part["text"] = command_to_process;
  user_parts.push_back(text_part);

  // 2) 如果 executor 提供了俯瞰图，把图像转成 base64 内联给 Gemini
  if (!latest_context_image_.data.empty()) {
    std::string image_b64 = encodeImageToBase64(latest_context_image_);
    if (!image_b64.empty()) {
      json img_part;
      img_part["inline_data"] = {
        {"mime_type", "image/png"},
        {"data", image_b64}
      };
      user_parts.push_back(img_part);
      ROS_INFO("[LLMPlanner] Added context image to request.");
    } else {
      ROS_WARN("[LLMPlanner] Context image present but base64 encoding failed, sending text only.");
    }
  } else {
    ROS_INFO("[LLMPlanner] No context image available, sending text only.");
  }

  gemini_contents.push_back({
    {"role", "user"},
    {"parts", user_parts}
  });
}
 else { // TOOL_CALLS
    if (!has_cached_plan_.load()) {
      publishFeedback("No cached plan to execute. Please run reasoning first.");
      return;
    }

    // [CHANGED] Use latest context text from executor (instead of calling getSwarmStateAsText here)
    std::string ctx = latest_context_text_;

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
      "- \"Go to the best approach point for [POI].\" → go_to_waypoint {drone_id, target_name:[POI]}\n"
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
    request_body["toolConfig"] = { {"functionCallingConfig", {{"mode", "ANY"}}} };
    request_body["tools"] = { getGeminiToolDefinitions() };
  }

  // System instruction
  std::string final_system_prompt;
  if (mode == RequestMode::REASONING_ONLY) {
    // [CHANGED] Use latest_context_text_ instead of calling executor's state
    final_system_prompt = system_prompt_content_ + "\n" + latest_context_text_ + R"(
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
  } else {
    final_system_prompt = R"(
You are a translator. Convert the provided plan into tool calls using the given tool schema.
Do NOT rethink. Do NOT change order or content. Return only function calls (no natural language).
)";
  }

  request_body["systemInstruction"] = {
    {"parts", {{{"text", final_system_prompt}}}}
  };
  request_body["generationConfig"] = { {"temperature", 0.0} };

  //ROS_INFO_STREAM("--- Sending Gemini Request ---\n" << request_body.dump(2));

  std::string model_name = "gemini-1.5-pro";
  std::string api_endpoint = "https://api.zhizengzeng.com/google/v1beta/models/"
      + model_name + ":streamGenerateContent?key=" + api_key_;

  cpr::WriteCallback write_callback{[this](const std::string_view& data, intptr_t /*userdata*/) -> bool {
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

  if (interrupt_llm_request_.load()) return;

  if (r.status_code == 200) {
    if (current_mode == RequestMode::REASONING_ONLY) {
      last_reasoning_text_ = accumulated_reasoning_response_;
      last_plan_text_      = extractPlanFromReasoning(last_reasoning_text_);
      has_cached_plan_.store(!last_plan_text_.empty());

      if (!accumulated_reasoning_response_.empty()) {
        publishReasoning(accumulated_reasoning_response_);
      }
      publishFeedback("Reasoning complete.");
      return;
    }

    if (!accumulated_tool_calls_.empty()) {
      //ROS_INFO_STREAM("Final collected tool calls: " << accumulated_tool_calls_.dump(2));
      publishFeedback("New plan received. Publishing tool calls to executor...");

      // [ADDED] Convert accumulated_tool_calls_ JSON → Plan.msg and publish
      nlp_drone_control::Plan plan_msg;
      plan_msg.replan_mode = static_cast<uint8_t>(ReplanMode::REPLACE); // mirror your previous behavior

      for (const auto& tc : accumulated_tool_calls_) {
        nlp_drone_control::ToolCall t;
        t.function_name = tc.at("function").at("name").get<std::string>();
        const auto& arg_node = tc.at("function").at("arguments");
        t.arguments_json = arg_node.is_string() ? arg_node.get<std::string>() : arg_node.dump();
        plan_msg.tool_calls.push_back(t);
      }
      plan_pub_.publish(plan_msg);

    } else {
      ROS_ERROR_STREAM("No tool calls parsed! Last raw reasoning: " << accumulated_reasoning_response_);
      publishFeedback("LLM finished without a clear action.");
    }
  } else {
    ROS_ERROR_STREAM("LLM API request failed with status " << r.status_code << ": " << r.text);
    publishFeedback("Error: LLM API request failed. Status: " + std::to_string(r.status_code));
  }
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
    {"description", "Commands a specific drone to fly to the best approach point of a named target (POI)."},
    {"parameters", {
        {"type", "object"},
        {"properties", {
            {"drone_id", {{"type", "string"}, {"description", "The drone ID to perform this action."}}},
            {"target_name", {{"type", "string"}, {"description", "The exact name of the POI to go to, e.g., 'House6', 'Church'."}}}
        }},
        {"required", json::array({"drone_id", "target_name"})}
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

void LLMPlannerNode::publishFeedback(const std::string& text) {
  std_msgs::String msg; msg.data = text;
  central_nlp_feedback_pub_.publish(msg);
}
void LLMPlannerNode::publishReasoning(const std::string& text) {
  std_msgs::String msg; msg.data = text;
  reasoning_pub_.publish(msg);
}

