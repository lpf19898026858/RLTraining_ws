#include "nlp_drone_control/task_executor_node.h"
#include <fstream>
#include <sstream>

// ================== Constructor / Destructor ==================

TaskExecutorNode::TaskExecutorNode(ros::NodeHandle& nh) : nh_(nh) {
  ros::NodeHandle pnh("~");
  ROS_INFO("Task Executor Node is initializing...");

  // Load swarm params: drone_ids_ and image_save_base_path_ and POIs
  pnh.getParam("drone_ids", drone_ids_);
  if (drone_ids_.empty()) {
    ROS_FATAL("Private parameter 'drone_ids' is not set. Executor needs to know which drones to manage.");
    ros::shutdown();
    return;
  }
  pnh.param<std::string>("image_save_base_path", image_save_base_path_, "/tmp/drone_images");
  // ========== NEW: connect to poi_state_server ==========
  list_pois_client_ = nh_.serviceClient<poi_state_server::ListPOIs>("/poi_state_server/list_pois");
  get_poi_info_client_ = nh_.serviceClient<poi_state_server::GetPOIInfo>("/poi_state_server/get_poi_info");
  
  // Per-drone ROS I/O and contexts
  for (const std::string& drone_id : drone_ids_) {
    drone_contexts_[drone_id].id = drone_id;
    // [Optional] 可尝试从 poi_state_server 获取该无人机的起始位置
    poi_state_server::GetPOIInfo srv;
    srv.request.name = drone_id;
    if (get_poi_info_client_.call(srv) && srv.response.success) {
      drone_contexts_[drone_id].pose.pose.position = srv.response.info.position;
      drone_contexts_[drone_id].launch_position = srv.response.info.position;
    }

    drone_pose_subs_.push_back(nh_.subscribe<geometry_msgs::PoseStamped>("/" + drone_id + "/drone_pose", 1, boost::bind(&TaskExecutorNode::dronePoseCallback, this, _1, drone_id)));
    drone_status_subs_.push_back(nh_.subscribe<std_msgs::String>("/" + drone_id + "/drone_status", 1, boost::bind(&TaskExecutorNode::droneStatusCallback, this, _1, drone_id)));
    action_feedback_subs_.push_back(nh_.subscribe<std_msgs::String>("/" + drone_id + "/action_feedback", 1, boost::bind(&TaskExecutorNode::actionFeedbackCallback, this, _1, drone_id)));

    execute_action_clients_[drone_id] = nh_.serviceClient<nlp_drone_control::ExecuteDroneAction>("/" + drone_id + "/execute_drone_action");
    llm_goal_pubs_[drone_id] = nh_.advertise<geometry_msgs::PoseStamped>("/" + drone_id + "/llm_goal", 1);
  }

  // Central interfaces (executor side)
  describe_scene_client_     = nh_.serviceClient<vlm_service::DescribeScene>("/vision_service/describe_scene");
  capture_image_client_      = nh_.serviceClient<vlm_service::CaptureImage>("/vision_service/capture_image");

  // Planner → Executor
  planner_tool_calls_sub_    = nh_.subscribe("/planner/tool_calls", 1, &TaskExecutorNode::plannerPlanCallback, this);

  // UI stop (executor side also listens)
  stop_command_sub_          = nh_.subscribe("/nlp_stop", 1, &TaskExecutorNode::stopCommandCallback, this);

  // Feedback / reasoning (same topics)
  central_nlp_feedback_pub_  = nh_.advertise<std_msgs::String>("/nlp_feedback", 10);
  reasoning_pub_             = nh_.advertise<std_msgs::String>("/nlp_reasoning", 10);

  // [ADDED] Periodic context text publisher for planner
  fleet_context_pub_         = nh_.advertise<std_msgs::String>("/fleet/context_text", 1);
  context_timer_             = nh_.createTimer(ros::Duration(1.0), &TaskExecutorNode::onContextTimer, this);

  // Start per-drone execution threads
  for (const std::string& drone_id : drone_ids_) {
    execution_threads_[drone_id] = std::thread(&TaskExecutorNode::executionLoop, this, drone_id);
  }
  
  // 加载俯瞰图
std::string overview_image_path;
pnh.param<std::string>("overview_image_path", overview_image_path, "/home/lpf/docker_shared/rltraining_ws/src/nlp_drone_control/config/overview.png");

overview_image_ = cv::imread(overview_image_path, cv::IMREAD_COLOR);
if (overview_image_.empty()) {
  ROS_ERROR_STREAM("Failed to load overview image from " << overview_image_path);
} else {
  ROS_INFO_STREAM("Loaded overview image from " << overview_image_path);
}

// 发布器
fleet_context_image_pub_ = nh_.advertise<sensor_msgs::Image>("/fleet/context_image", 1);

  ROS_INFO("Task Executor Node is ready.");
}

TaskExecutorNode::~TaskExecutorNode() {
  for (auto& pair : execution_threads_) {
    if (pair.second.joinable()) pair.second.join();
  }
}

// ================== Planner Plan Callback ==================

void TaskExecutorNode::plannerPlanCallback(const nlp_drone_control::Plan::ConstPtr& msg) {
  // Convert Plan.msg back to JSON array (compatible with your old dispatchToolCalls)
  json tool_calls = json::array();
  for (const auto& call : msg->actions) {
    json tc;
    tc["function"]["name"] = call.function_name;
    // arguments_json is a string; keep as string to match old code path
    tc["function"]["arguments"] = call.arguments_json;
    tool_calls.push_back(tc);
  }

  ReplanMode mode = static_cast<ReplanMode>(msg->replan_mode);
  dispatchToolCalls(tool_calls, mode);
}

// ================== Stop / Drone status callbacks (verbatim move) ==================

void TaskExecutorNode::stopCommandCallback(const std_msgs::Empty::ConstPtr& /*msg*/) {
  ROS_WARN("LLM STOP command received at Executor! Clearing queues.");
  // Clear all task queues safely
  std::lock_guard<std::mutex> lock(contexts_map_mutex_);
  for (auto& kv : drone_contexts_) {
    auto& ctx = kv.second;
    std::lock_guard<std::mutex> ctx_lock(ctx.context_data_mutex);
    ctx.tool_call_queue.clear();
    ctx.is_task_executing = false;
    ctx.is_in_macro_task = false;
  }
}

void TaskExecutorNode::dronePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, const std::string& drone_id) {
  std::lock_guard<std::mutex> lock(contexts_map_mutex_);
  if (drone_contexts_.count(drone_id)) drone_contexts_.at(drone_id).pose = *msg;
}

void TaskExecutorNode::droneStatusCallback(const std_msgs::String::ConstPtr& msg, const std::string& drone_id) {
  bool should_handle_completion = false;
  {
    std::lock_guard<std::mutex> lock(contexts_map_mutex_);
    if (!drone_contexts_.count(drone_id)) {
      ROS_WARN_STREAM("Received status from unknown drone_id: " << drone_id);
      return;
    }
    auto& context = drone_contexts_.at(drone_id);
    DroneState old_state = context.state_enum;

    context.status_str = msg->data;
    if      (context.status_str == "IDLE_ON_GROUND") context.state_enum = GROUNDED;
    else if (context.status_str == "NAVIGATING")     context.state_enum = FLYING;
    else if (context.status_str == "HOVERING" || context.status_str == "IDLE_IN_AIR") context.state_enum = HOVERING;
    else if (context.status_str == "PERFORMING_ACTION") context.state_enum = PERFORMING_ACTION;
    else if (context.status_str == "TAKING_OFF")     context.state_enum = TAKING_OFF;
    else if (context.status_str == "LANDING")        context.state_enum = LANDING;
    else                                             context.state_enum = UNKNOWN;

    DroneState new_state = context.state_enum;
    if ((old_state == FLYING && new_state == HOVERING) ||
        (old_state == TAKING_OFF && new_state == HOVERING) ||
        (old_state == LANDING && new_state == GROUNDED) ||
        (old_state == PERFORMING_ACTION && new_state == HOVERING)) {
      should_handle_completion = true;
    }
  }

  if (should_handle_completion) {
    handle_task_completion(drone_id);
  }
}

void TaskExecutorNode::actionFeedbackCallback(const std_msgs::String::ConstPtr& msg, const std::string& drone_id) {
  if (msg->data == "ACTION_COMPLETE") {
    handle_task_completion(drone_id);
  }
}

// ================== Context text timer (ADDED) ==================

void TaskExecutorNode::onContextTimer(const ros::TimerEvent&) {
  //std_msgs::String msg;
  //msg.data = getSwarmStateAsText();
  //fleet_context_pub_.publish(msg);
    // 1) 发布简短文本
  std_msgs::String msg;
  msg.data = "There are currently " + std::to_string(drone_ids_.size()) + " drones. Please use the overhead view for inference.";
  fleet_context_pub_.publish(msg);

  // 2) 发布俯瞰图
  if (!overview_image_.empty()) {
    sensor_msgs::Image ros_img;
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "map";

    cv_bridge::CvImage cv_img(header, "bgr8", overview_image_);
    fleet_context_image_pub_.publish(*cv_img.toImageMsg());
  }
}

// ================== State text (verbatim move) ==================

std::string TaskExecutorNode::getSwarmStateAsText() {
  std::string dynamic_info_str;
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

  dynamic_info_str += "\n--- Points of Interest (POIs) ---\n";
  poi_state_server::ListPOIs srv;
  if (list_pois_client_.call(srv) && srv.response.success) {
    for (const auto& poi : srv.response.pois) {
      dynamic_info_str += "- Name: " + poi.name + "\n";
      dynamic_info_str += "  Type: " + poi.type + "\n";
      dynamic_info_str += "  Ref Coordinates: (x:" + std::to_string(poi.position.x) +
                          ", y:" + std::to_string(poi.position.y) +
                          ", z:" + std::to_string(poi.position.z) + ")\n";
    }
  } else {
    dynamic_info_str += "No POIs available (poi_state_server not responding).\n";
  }
  return dynamic_info_str;
}

// ================== Dispatch & Execution (verbatim move) ==================

void TaskExecutorNode::dispatchToolCalls(const json& tool_calls, ReplanMode mode) {
  std::map<std::string, std::vector<json>> tasks_by_drone;
  for (const auto& tool_call : tool_calls) {
    try {
      const auto& arg_node = tool_call.at("function").at("arguments");
      json args = arg_node.is_string() ? json::parse(arg_node.get<std::string>()) : arg_node;
      std::string drone_id = args.at("drone_id").get<std::string>();
      tasks_by_drone[drone_id].push_back(tool_call);
    } catch (const std::exception& e) {
      ROS_ERROR("Error parsing tool call for dispatch: %s", e.what());
    }
  }

  std::vector<std::string> drones_to_interrupt;
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
          context.tool_call_queue.clear();
          if (needs_interrupt) {
            ROS_INFO("Injecting a 1.0s WAIT command to stabilize drone %s after interrupt.", drone_id.c_str());
            json wait_tool;
            wait_tool["function"]["name"] = "wait";
            json wait_args;
            wait_args["drone_id"] = drone_id;
            wait_args["duration_seconds"] = 1.0;
            wait_tool["function"]["arguments"] = wait_args.dump();
            context.tool_call_queue.push_front(wait_tool);
            drones_to_interrupt.push_back(drone_id);
          }
        }
        for (const auto& task : new_tasks) {
          context.tool_call_queue.push_back(task);
        }
        ROS_INFO("Enqueued %zu new tasks for drone %s.", context.tool_call_queue.size(), drone_id.c_str());
      }
    }
  }
  for (const auto& drone_id : drones_to_interrupt) {
    publishFeedback("Interrupting " + drone_id + " for new high-priority task.");
    send_simple_command(drone_id, "wait", json{{"duration", 0.1}}.dump());
  }
}

void TaskExecutorNode::executionLoop(const std::string& drone_id) {
  ros::Rate rate(5);
  while (ros::ok()) {
    {
      std::lock_guard<std::mutex> lock(contexts_map_mutex_);
      auto& context = drone_contexts_.at(drone_id);
      const ros::Duration NAVIGATION_TIMEOUT(60.0);
      if (context.state_enum == FLYING && !context.navigation_start_time_.is_zero() &&
          (ros::Time::now() - context.navigation_start_time_ > NAVIGATION_TIMEOUT)) {
        ROS_ERROR("Drone %s navigation timed out!", drone_id.c_str());
        publishFeedback("Error: Plan for " + drone_id + " aborted due to navigation timeout.");

        std::lock_guard<std::mutex> context_lock(context.context_data_mutex);
        context.tool_call_queue.clear();
        context.is_task_executing = false;

        send_simple_command(drone_id, "hover");
        context.navigation_start_time_ = ros::Time(0);
      }
    }

    bool can_start_new_task = false;
    {
      std::lock_guard<std::mutex> lock(contexts_map_mutex_);
      auto& context = drone_contexts_.at(drone_id);
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
        context.is_task_executing = true;
      }

      std::string result = executeTool(next_tool, drone_id);

      if (result.rfind("Error:", 0) == 0) {
        std::lock_guard<std::mutex> lock(contexts_map_mutex_);
        auto& context = drone_contexts_.at(drone_id);
        std::lock_guard<std::mutex> context_lock(context.context_data_mutex);
        context.is_task_executing = false;
        context.tool_call_queue.clear();
        publishFeedback("Plan for " + drone_id + " aborted due to command error: " + result);
      }
    }
    rate.sleep();
  }
}

// ================== Tool execution & helpers (verbatim move) ==================

void TaskExecutorNode::publishGoal(const geometry_msgs::PoseStamped& goal_pose, const std::string& drone_id) {
  if (llm_goal_pubs_.count(drone_id)) {
    llm_goal_pubs_.at(drone_id).publish(goal_pose);
  }
}

std::string TaskExecutorNode::go_to_waypoint_action(double x, double y, double z, const std::string& drone_id) {
  {
    std::lock_guard<std::mutex> lock(contexts_map_mutex_);
    auto& context = drone_contexts_.at(drone_id);
    if (context.state_enum != HOVERING) return "Error: Drone must be hovering.";
    context.navigation_start_time_ = ros::Time::now();
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

std::string TaskExecutorNode::executeTool(const json& tool_call, const std::string& drone_id) {
  std::string tool_name;
  json tool_input;
  try {
    tool_name = tool_call.at("function").at("name").get<std::string>();
    const auto& arg_node = tool_call.at("function").at("arguments");
    tool_input = arg_node.is_string() ? json::parse(arg_node.get<std::string>()) : arg_node;
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
  if (!tool_input.contains("target_name")) {
    result_msg = "Error: 'target_name' is required for go_to_waypoint.";
  } else {
    std::string target = tool_input["target_name"].get<std::string>();
geometry_msgs::Point target_pos;
if (_pois.find(target) != _pois.end()) {
    if (_pois.at(target).has_best_approach) {
        target_pos = _pois.at(target).best_approach;
    } else {
        target_pos = _pois.at(target).position;
    }
} else {
  poi_state_server::GetPOIInfo srv;
  srv.request.name = target;
if (get_poi_info_client_.call(srv) && srv.response.success) {
    POIInfoData data;
    data.name = srv.response.info.name;
    data.type = srv.response.info.type;
    data.description = srv.response.info.description;
    data.position = srv.response.info.position;
    data.candidate_points.clear();
    for (const auto& cp : srv.response.info.candidate_points) {
        NamedPoint np;
        np.name  = cp.name;
        np.point = cp.point;
        data.candidate_points.push_back(np);
    }

    data.has_simplified_boundary = srv.response.info.has_boundary;
    data.simplified_boundary = {srv.response.info.boundary_min, srv.response.info.boundary_max};

    if (!srv.response.info.candidate_points.empty()) {
        data.best_approach = srv.response.info.candidate_points[0].point;
        data.has_best_approach = true;
        ROS_INFO_STREAM("Using best_approach for POI " << data.name
                        << " at (" << data.best_approach.x << ", "
                        << data.best_approach.y << ", "
                        << data.best_approach.z << ")");
    } else {
        data.has_best_approach = false;
        ROS_INFO_STREAM("No best_approach available for POI " << data.name
                        << ", fallback to position.");
    }

    // ✅ 缓存
    _pois[target] = data;

    // ✅ 这里才决定 target_pos
    if (data.has_best_approach) {
        target_pos = data.best_approach;
    } else {
        target_pos = data.position;
    }
}
else {
    result_msg = "Error: POI '" + target + "' not found in poi_state_server.";
    publishFeedback(result_msg);
    return result_msg;
  }
}
    // 调用 go_to_waypoint_action 发布目标
    result_msg = go_to_waypoint_action(target_pos.x, target_pos.y, target_pos.z, drone_id);
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
  } else if (tool_name == "perform_visual_inspection") {
    if (tool_input.contains("target_name")) {
      start_visual_inspection_macro(tool_input["target_name"].get<std::string>(), drone_id);
      result_msg = "Starting visual inspection macro-task for " + drone_id;
    } else {
      result_msg = "Error: 'target_name' is missing for perform_visual_inspection.";
    }
  } else if (tool_name == "rotate_drone_yaw_relative") {
    result_msg = send_simple_command(drone_id, "rotate_drone_yaw_relative", tool_input.dump());
  } else if (tool_name == "backup") {
    result_msg = send_simple_command(drone_id, "backup", tool_input.dump());
  } else if (tool_name == "adjust_camera_pitch") {
    result_msg = send_simple_command(drone_id, "adjust_camera_pitch", tool_input.dump());
  } else {
    result_msg = "Error: Unknown or unsupported tool: " + tool_name;
  }

  if (result_msg.rfind("Error:", 0) != 0) {
    publishFeedback("LLM Action [" + drone_id + "]: " + result_msg);
  }
  return result_msg;
}

std::string TaskExecutorNode::send_simple_command(const std::string& drone_id, const std::string& action_type, const std::string& params_json) {
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

std::string TaskExecutorNode::cancel_all_tasks_action(const std::string& drone_id) {
  ROS_WARN("Executing cancel_all_tasks for drone %s", drone_id.c_str());
  {
    std::lock_guard<std::mutex> lock(contexts_map_mutex_);
    auto& context = drone_contexts_.at(drone_id);
    context.is_in_macro_task = false;
    std::lock_guard<std::mutex> context_lock(context.context_data_mutex);
    context.tool_call_queue.clear();
  }
  send_simple_command(drone_id, "wait", json{{"duration_seconds", 1.0}}.dump());
  return "All tasks for " + drone_id + " cancelled. Drone is returning to hover state.";
}

std::string TaskExecutorNode::return_to_launch_action(const std::string& drone_id) {
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
    return go_to_waypoint_action(launch_pos.x, launch_pos.y, 2.0, drone_id);
  }
  return "Error: Should not happen, failed to retrieve launch position.";
}

// ================== Macro task (verbatim move) ==================

void TaskExecutorNode::handle_task_completion(const std::string& drone_id) {
  NextMacroAction next_action;
  {
    std::lock_guard<std::mutex> lock(contexts_map_mutex_);
    if (!drone_contexts_.count(drone_id)) return;

    auto& context = drone_contexts_.at(drone_id);
    if (context.is_in_macro_task) {
      ROS_INFO("[%s] Macro sub-step complete. Advancing macro...", drone_id.c_str());
      next_action = advance_visual_inspection_macro(drone_id);
    } else {
      std::lock_guard<std::mutex> context_lock(context.context_data_mutex);
      context.is_task_executing = false;
      ROS_INFO("[%s] Simple task complete.", drone_id.c_str());
    }
  }

  if (next_action.is_valid) {
    if (next_action.is_capture_action) {
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
        handle_task_completion(drone_id);
      } else {
        publishFeedback("Error: [" + drone_id + "] Failed to capture image. Aborting inspection.");
        std::lock_guard<std::mutex> lock(contexts_map_mutex_);
        auto& context = drone_contexts_.at(drone_id);
        context.is_in_macro_task = false;
        context.is_task_executing = false;
      }
    } else if (next_action.action_type == "ANALYZE_VLM") {
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
      handle_task_completion(drone_id);
    } else {
      send_simple_command(drone_id, next_action.action_type, next_action.params_json);
    }
  }
}

double TaskExecutorNode::calculate_yaw_to_target(const std::string& target_name, const geometry_msgs::PoseStamped& drone_pose) {
  geometry_msgs::Point target_pos = _pois.at(target_name).position;

  tf2::Quaternion q_ros;
  tf2::fromMsg(drone_pose.pose.orientation, q_ros);
  tf2::Vector3 forward_vec_ros_3d = tf2::Transform(q_ros) * tf2::Vector3(1, 0, 0);
  tf2::Vector3 target_dir_ros_3d(target_pos.x - drone_pose.pose.position.x,
                                 target_pos.y - drone_pose.pose.position.y,
                                 target_pos.z - drone_pose.pose.position.z);
  tf2::Vector3 forward_vec_ros_2d = forward_vec_ros_3d; forward_vec_ros_2d.setZ(0);
  if (forward_vec_ros_2d.length() < 0.001) {
    ROS_WARN("Drone is pointing vertically. Using a default forward direction for yaw calculation.");
    forward_vec_ros_2d.setValue(1.0, 0.0, 0.0);
  }
  forward_vec_ros_2d.normalize();
  tf2::Vector3 target_dir_ros_2d = target_dir_ros_3d; target_dir_ros_2d.setZ(0);
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

void TaskExecutorNode::start_visual_inspection_macro(const std::string& target_name, const std::string& drone_id) {
  NextMacroAction first_action;
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
    context.macro_task_step = 0;

    context.macro_task_data.clear();
    context.macro_task_data["target_name"] = target_name;
    context.collected_images_for_vlm.clear();

    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss; ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
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

    first_action = advance_visual_inspection_macro(drone_id);
  }

  if (first_action.is_valid) {
    send_simple_command(drone_id, first_action.action_type, first_action.params_json);
  }
}

NextMacroAction TaskExecutorNode::advance_visual_inspection_macro(const std::string& drone_id) {
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
  next_action.is_valid = true;

  switch (context.macro_task_step) {
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
          return advance_visual_inspection_macro(drone_id);
        }
      }
      break;
    case 3:
      publishFeedback("[" + drone_id + "] Inspection Step 3/13: Backing up (2m)...");
      next_action.action_type = "backup";
      next_action.params_json = json{{"distance", 2.0}}.dump();
      break;
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
    case 14:
      publishFeedback("[" + drone_id + "] Inspection Step 14/15: Returning to center yaw...");
      next_action.action_type = "rotate_drone_yaw_relative";
      next_action.params_json = json{{"angle_degrees", -INSPECTION_YAW_ANGLE}}.dump();
      break;
    case 15:
      publishFeedback("[" + drone_id + "] Inspection Step 15/15: All images captured. Requesting VLM analysis...");
      next_action.action_type = "ANALYZE_VLM";
      break;
    default:
      publishFeedback("[" + drone_id + "] Visual Inspection Macro COMPLETED.");
      {
        std::lock_guard<std::mutex> context_lock(context.context_data_mutex);
        context.is_in_macro_task = false;
        context.is_task_executing = false;
        context.collected_images_for_vlm.clear();
        context.macro_task_data.clear();
      }
      next_action.is_valid = false;
      break;
  }
  return next_action;
}

bool TaskExecutorNode::capture_and_get_image(const std::string& dir, const std::string& filename, sensor_msgs::Image& out_image, const std::string& drone_id) {
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

void TaskExecutorNode::publishFeedback(const std::string& text) {
  std_msgs::String msg; msg.data = text;
  central_nlp_feedback_pub_.publish(msg);
}

tf2::Vector3 TaskExecutorNode::transformRosToUnity(const tf2::Vector3& ros_vec) {
  return tf2::Vector3(
    -ros_vec.y(), // Unity X is ROS -Y
     ros_vec.z(), // Unity Y is ROS +Z
     ros_vec.x()  // Unity Z is ROS +X
  );
}

