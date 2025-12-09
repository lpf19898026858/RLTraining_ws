#include <ros/ros.h>
#include <map>
#include <string>
#include <vector>
#include <deque>
#include <mutex>

#include "nlp_drone_control/Plan.h"
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/String.h"
#include "fleet_manager/QueryFleetAndTasks.h"
#include "fleet_manager/InjectTasks.h"
#include "uav_scheduler/ComputeAssignment.h"          // <<< 调度器服务
#include <nlohmann/json.hpp>
using json = nlohmann::json;
// 头部
#include <deque>

struct TaskInfo {
    int id;
    fleet_manager::TaskStatus status_msg;
};

struct DroneInfo {
    fleet_manager::DroneStatus status_msg;
    ros::Time      last_heartbeat;
    ros::WallTime  last_heartbeat_wall;
    bool           seen_first_hb = false;
    std::vector<int> assigned_task_ids;
};

// 每架无人机的一份“POI待办队列”
struct DronePlanMirror {
    std::deque<std::string> poi_queue;  // 还没inspect的POI，按顺序
};

class FleetManager {
public:
    FleetManager(ros::NodeHandle& nh) : nh_(nh) {
        // 1) 订阅 Planner 输出
        plan_sub_ = nh_.subscribe("/planner/tool_calls", 1, &FleetManager::planCallback, this);

        // 2) 读取 drone_ids
        std::vector<std::string> drone_ids_param;
        if (!nh_.getParam("drone_ids", drone_ids_param) || drone_ids_param.empty()) {
            ROS_ERROR("Param 'drone_ids' not set or empty. Watchdog will monitor NOTHING.");
        } else {
            drone_ids_ = drone_ids_param;
        }

        // 3) 看门狗参数 + WallTimer
        nh_.param("heartbeat_timeout_ms", heartbeat_timeout_ms_, heartbeat_timeout_ms_);
        nh_.param("watchdog_period_ms",   watchdog_period_ms_,   watchdog_period_ms_);
        nh_.param<std::string>("scheduler_algorithm", scheduler_algorithm_, std::string("Hungarian Algorithm"));
        watchdog_wall_timer_ = nh_.createWallTimer(
            ros::WallDuration(watchdog_period_ms_ / 1000.0),
            &FleetManager::watchdogWallCallback, this);

        // 4) 为每个无人机建订阅并初始化状态
        for (const auto& drone_id : drone_ids_) {
            feedback_subs_.push_back(
                nh_.subscribe<std_msgs::String>("/"+drone_id+"/action_feedback", 10,
                    boost::bind(&FleetManager::actionFeedbackCallback, this, _1, drone_id)));
            heartbeat_subs_.push_back(
                nh_.subscribe<geometry_msgs::PoseStamped>("/"+drone_id+"/drone_pose", 1,
                    boost::bind(&FleetManager::heartbeatCallback, this, _1, drone_id)));

            DroneInfo& info = drone_info_[drone_id];
            info.status_msg.drone_id = drone_id;
            info.status_msg.status = fleet_manager::DroneStatus::ACTIVE;
            info.status_msg.last_completed_task_index = -1;
            info.last_heartbeat_wall = ros::WallTime::now();
            info.seen_first_hb = false;

            // 初始化镜像
            (void)poi_plan_[drone_id];
            ROS_INFO("[FleetManager] Monitoring drone %s", drone_id.c_str());
        }

        // 5) 服务
        status_service_   = nh_.advertiseService("query_fleet_and_tasks", &FleetManager::queryStatusCallback, this);
        injector_service_ = nh_.advertiseService("inject_tasks", &FleetManager::injectCallback, this);

        // 6) 发布器
        plan_pub_         = nh_.advertise<nlp_drone_control::Plan>("/planner/tool_calls", 10);

        // 7) 调度器客户端
        scheduler_client_ = nh_.serviceClient<uav_scheduler::ComputeAssignment>("compute_assignment");

hl_event_sub_ = nh_.subscribe<std_msgs::String>(
    "/fleet/hl_event", 10, &FleetManager::hlEventCallback, this);

        ROS_INFO("Fleet Manager Node is ready.");
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher plan_pub_;
    ros::ServiceServer status_service_;
    ros::ServiceServer injector_service_;
    ros::Subscriber plan_sub_;
    std::vector<ros::Subscriber> feedback_subs_;
    std::vector<ros::Subscriber> heartbeat_subs_;
    std::mutex mtx_;

    std::map<int, TaskInfo> all_tasks_;
    std::map<std::string, DroneInfo> drone_info_;
    int next_task_id_ = 0;
    std::vector<std::string> drone_ids_;
    
std::map<std::string, std::deque<std::string>> poi_queues_;
ros::Subscriber hl_event_sub_;

    // 每机的POI队列镜像
    std::map<std::string, DronePlanMirror> poi_plan_;

    // 计时器/调度器
    ros::WallTimer watchdog_wall_timer_;
    int heartbeat_timeout_ms_ = 10000;
    int watchdog_period_ms_   = 2000;
    std::string scheduler_algorithm_ = "Hungarian Algorithm";
    ros::ServiceClient scheduler_client_;

    // ---------- 小工具 ----------
    static inline std::string actionNameOf(const nlp_drone_control::Action& a) {
        return a.function_name; // 如果字段名不同，这里改
    }
    static inline std::string extractTargetName(const nlp_drone_control::Action& a) {
        try {
            json j = json::parse(a.arguments_json);
            if (j.contains("target_name")) return j["target_name"].get<std::string>();
        } catch (...) {}
        return "";
    }
    static inline nlp_drone_control::Action makeAction(const std::string& name, const json& args) {
        nlp_drone_control::Action a;
        a.function_name = name;
        a.arguments_json = args.dump();
        return a;
    }

    // ---------- 回调 ----------
    void planCallback(const nlp_drone_control::Plan::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        ROS_INFO("[StatusServer] Received a new plan with %zu actions.", msg->actions.size());

        // 1) 生成 TaskInfo（保持你的原逻辑）
        for (const auto& action : msg->actions) {
            std::string drone_id;
            json args;
            try {
                args = json::parse(action.arguments_json);
                drone_id = args.at("drone_id").get<std::string>();
            } catch (const std::exception& e) {
                ROS_ERROR("planCallback: bad arguments_json, skip action. err=%s", e.what());
                continue;
            }

            if (!drone_info_.count(drone_id)) {
                ROS_WARN("planCallback: drone_id '%s' not monitored; task accepted but no heartbeat subscription exists.", drone_id.c_str());
            }

            int task_id = next_task_id_++;
            TaskInfo new_task;
            new_task.id = task_id;
            new_task.status_msg.task_id = task_id;
            new_task.status_msg.action_details = action;
            new_task.status_msg.status = fleet_manager::TaskStatus::PENDING;
            new_task.status_msg.assigned_drone_id = drone_id;

            all_tasks_[task_id] = new_task;
            drone_info_[drone_id].assigned_task_ids.push_back(task_id);
        }
    bool is_replace = (msg->replan_mode == 1);
        // 2) 同步 POI 队列镜像：把所有 "perform_visual_inspection" 的 target_name 依次加入队列
        //    （APPEND 语义；如果未来支持 REPLACE，可在此清空后重建）
        std::map<std::string, std::vector<std::string>> poi_to_append;
        for (const auto& action : msg->actions) {
            if (actionNameOf(action) != "perform_visual_inspection") continue;
            std::string drone_id, poi;
            try {
                json args = json::parse(action.arguments_json);
                drone_id = args.at("drone_id").get<std::string>();
                poi      = args.at("target_name").get<std::string>();
            } catch (...) { continue; }
            //poi_to_append[drone_id].push_back(poi);
        }
            // REPLACE：先清空被触及无人机的队列
    if (is_replace) {
        for (const auto& kv : per_drone_pois) {
            poi_plan_[kv.first].poi_queue.clear();
        }
    }

    // 逐个 APPEND 进去
    for (auto& kv : per_drone_pois) {
        auto& q = poi_plan_[kv.first].poi_queue;
        for (const auto& poi : kv.second) {
            q.push_back(poi);
            ROS_INFO("[POI-QUEUE] [%s] + %s  (size=%zu)", kv.first.c_str(), poi.c_str(), q.size());
        }
    }
    }

    void heartbeatCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, const std::string& drone_id) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (drone_info_.count(drone_id)) {
            drone_info_[drone_id].last_heartbeat      = ros::Time::now();
            drone_info_[drone_id].last_heartbeat_wall = ros::WallTime::now();
            drone_info_[drone_id].status_msg.last_known_pose = msg->pose;
            drone_info_[drone_id].seen_first_hb = true;
        }
    }

    void actionFeedbackCallback(const std_msgs::String::ConstPtr& msg, const std::string& drone_id) {
        if (msg->data != "ACTION_COMPLETE") return;

        std::lock_guard<std::mutex> lock(mtx_);
        if (!drone_info_.count(drone_id)) return;

        auto& di = drone_info_.at(drone_id);
        size_t next_idx = static_cast<size_t>(di.status_msg.last_completed_task_index + 1);

        if (next_idx < di.assigned_task_ids.size()) {
            di.status_msg.last_completed_task_index = static_cast<int>(next_idx);
            int tid = di.assigned_task_ids[next_idx];
            if (all_tasks_.count(tid)) {
                auto& t = all_tasks_.at(tid);
                t.status_msg.status = fleet_manager::TaskStatus::COMPLETED;

                // 如果完成的是 "perform_visual_inspection"，从该机的 POI 队列里弹一个
                const auto& act = t.status_msg.action_details;
                if (actionNameOf(act) == std::string("perform_visual_inspection")) {
                    const std::string poi = extractTargetName(act);
                    auto& q = poi_plan_[drone_id].poi_queue;
                    if (!q.empty() && q.front() == poi) {
                        q.pop_front();
                        ROS_INFO("[POI-QUEUE] [%s] - %s (front pop)  size=%zu", drone_id.c_str(), poi.c_str(), q.size());
                    } else {
                        // 容错：如果顺序对不上，就删除队列中第一个匹配项
                        bool erased=false;
                        for (auto it=q.begin(); it!=q.end(); ++it) {
                            if (*it == poi) { q.erase(it); erased=true; break; }
                        }
                        ROS_WARN("[POI-QUEUE] [%s] - %s (%s) size=%zu",
                                 drone_id.c_str(), poi.c_str(), erased? "erase-first-match":"not-found", q.size());
                    }
                }
            }
            ROS_INFO("[StatusServer] Drone %s completed task index %zu (ID: %d).",
                     drone_id.c_str(), next_idx, tid);
        } else {
            ROS_WARN("[StatusServer] ACTION_COMPLETE from %s but no pending tasks (idx=%zu, size=%zu)",
                     drone_id.c_str(), next_idx, di.assigned_task_ids.size());
        }
    }
void hlEventCallback(const std_msgs::String::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mtx_);

    json j = json::parse(msg->data, nullptr, false);
    if (j.is_discarded()) return;

    // 只关心：type=INSPECT, stage=CAPTURED
    if (j.value("type","")  != "INSPECT")  return;
    if (j.value("stage","") != "CAPTURED") return;

    const std::string did = j.value("drone_id", "");
    const std::string poi = j.value("target",   "");
    if (did.empty() || poi.empty()) return;

    auto& q = poi_plan_[did].poi_queue;
    auto it = std::find(q.begin(), q.end(), poi);
    if (it != q.end()) {
        q.erase(it);
        ROS_INFO("[POI-QUEUE] [%s] - %s (pop by CAPTURED) size=%zu", did.c_str(), poi.c_str(), q.size());
    } else {
        ROS_WARN("[POI-QUEUE] [%s] CAPTURED for %s but queue has no such entry (maybe already popped).", did.c_str(), poi.c_str());
    }

    // （可选）把 all_tasks_ 中该 POI 的 perform_visual_inspection 标记 COMPLETED，便于查询/可视化
    for (auto& kv : all_tasks_) {
        auto& st = kv.second.status_msg;
        if (st.assigned_drone_id != did) continue;
        if (st.action_details.function_name != "perform_visual_inspection") continue;

        json a = json::parse(st.action_details.arguments_json, nullptr, false);
        if (a.is_discarded()) continue;
        if (a.value("target_name","") == poi) {
            st.status = fleet_manager::TaskStatus::COMPLETED;
        }
    }
}

    // 调 scheduler：active_drones × remaining_pois -> assignments "Drone->POI"
    bool callScheduler(const std::vector<std::string>& active_drones,
                       const std::vector<std::string>& pois,
                       std::map<std::string,std::vector<std::string>>& drone_to_pois)
    {
        drone_to_pois.clear();
        if (active_drones.empty() || pois.empty()) return false;

        uav_scheduler::ComputeAssignment srv;
        srv.request.drones = active_drones;
        srv.request.pois   = pois;
        srv.request.algorithm = scheduler_algorithm_;

        if (!scheduler_client_.call(srv)) {
            ROS_ERROR("Scheduler service call failed.");
            return false;
        }
        // 解析 "V_UAV_1->House2"
        for (const auto& line : srv.response.assignments) {
            auto p = line.find("->");
            if (p != std::string::npos) {
                std::string d   = line.substr(0,p);
                std::string poi = line.substr(p+2);
                drone_to_pois[d].push_back(poi);
            }
        }
        return !drone_to_pois.empty();
    }

    void watchdogWallCallback(const ros::WallTimerEvent&) {
        // 第一步：判超时（持锁）
        std::vector<std::string> failed_ids;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            const ros::WallTime now = ros::WallTime::now();

            for (auto& kv : drone_info_) {
                const std::string& id = kv.first;
                auto& d = kv.second;

                if (d.status_msg.status != fleet_manager::DroneStatus::ACTIVE) continue;
                if (!d.seen_first_hb) continue;

                double ms = (now - d.last_heartbeat_wall).toSec() * 1000.0;
                if (ms > heartbeat_timeout_ms_) {
                    ROS_ERROR("[Watchdog] Drone %s timed out (%.0f ms > %d ms). Marking NEEDS_BACKUP.",
                              id.c_str(), ms, heartbeat_timeout_ms_);
                    d.status_msg.status = fleet_manager::DroneStatus::NEEDS_BACKUP;
                    failed_ids.push_back(id);

                    // 标记其未完成的 task
                    size_t start = static_cast<size_t>(d.status_msg.last_completed_task_index + 1);
                    for (size_t i = start; i < d.assigned_task_ids.size(); ++i) {
                        int tid = d.assigned_task_ids[i];
                        if (all_tasks_.count(tid)) {
                            all_tasks_[tid].status_msg.status = fleet_manager::TaskStatus::NEEDS_BACKUP;
                        }
                    }
                }
            }
        }
        if (failed_ids.empty()) return;

        // 对每个失联无人机，拉取其剩余POI并调度
        for (const auto& failed : failed_ids) {
            std::vector<std::string> remaining_pois;
            std::vector<std::string> candidates;
            {
                std::lock_guard<std::mutex> lock(mtx_);
                // 剩余 POI = 队列全部（因为我们只在INSPECT完成时弹出）
                const auto& q = poi_plan_[failed].poi_queue;
                remaining_pois.assign(q.begin(), q.end());

                for (auto& kv : drone_info_) {
                    if (kv.first == failed) continue;
                    if (kv.second.status_msg.status == fleet_manager::DroneStatus::ACTIVE)
                        candidates.push_back(kv.first);
                }
            }
            if (remaining_pois.empty()) {
                ROS_WARN("[Watchdog] %s has no remaining POIs to reassign.", failed.c_str());
                continue;
            }
            if (candidates.empty()) {
                ROS_ERROR("[Watchdog] No ACTIVE candidates to reassign POIs of %s.", failed.c_str());
                continue;
            }

            std::map<std::string,std::vector<std::string>> assign; // drone -> {pois}
            if (!callScheduler(candidates, remaining_pois, assign)) {
                ROS_ERROR("[Watchdog] Scheduler returned empty assignment. Skip reassign for %s.", failed.c_str());
                continue;
            }

            // 组一个大 Plan：每个接盘无人机一段 "takeoff + (go_to+inspect)* + rtl + land"
            nlp_drone_control::Plan plan; plan.replan_mode = 0; // APPEND
            {
                std::lock_guard<std::mutex> lock(mtx_);

                // 把失败机的队列清空（避免后续误用），具体动作以新 plan 为准
                poi_plan_[failed].poi_queue.clear();

                for (auto& kv : assign) {
                    const std::string& to_drone = kv.first;
                    const auto& pois = kv.second;
                    if (pois.empty()) continue;

                    // takeoff
                    plan.actions.push_back(makeAction("takeoff", json{{"drone_id", to_drone}, {"target_altitude", 2.0}}));

                    // 每个POI：go_to + inspect
                    for (const auto& poi : pois) {
                        plan.actions.push_back(makeAction("go_to_waypoint",            json{{"drone_id", to_drone}, {"target_name", poi}}));
                        plan.actions.push_back(makeAction("perform_visual_inspection", json{{"drone_id", to_drone}, {"target_name", poi}}));
                    }

                    // 返回并降落
                    plan.actions.push_back(makeAction("return_to_launch", json{{"drone_id", to_drone}}));
                    plan.actions.push_back(makeAction("land",             json{{"drone_id", to_drone}}));

                    // 镜像：把这些 POI 追加到接盘机的队列（等 planCallback 再次确认也无妨）
                    auto& q = poi_plan_[to_drone].poi_queue;
                    for (const auto& poi : pois) q.push_back(poi);
                }
            }

            if (!plan.actions.empty()) {
                plan_pub_.publish(plan);
                ROS_WARN("[FleetManager] Reassigned %zu POIs from %s to %zu drones (total %zu actions).",
                         remaining_pois.size(), failed.c_str(), assign.size(), plan.actions.size());
            }
        }
    }

    bool queryStatusCallback(fleet_manager::QueryFleetAndTasks::Request &,
                             fleet_manager::QueryFleetAndTasks::Response &res) {
        std::lock_guard<std::mutex> lock(mtx_);
        for (const auto& pair : drone_info_) res.drones.push_back(pair.second.status_msg);
        for (const auto& pair : all_tasks_)  res.tasks.push_back(pair.second.status_msg);
        return true;
    }

    bool injectCallback(fleet_manager::InjectTasks::Request &req,
                        fleet_manager::InjectTasks::Response &res) {
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (drone_info_.find(req.assign_to_drone_id) == drone_info_.end() ||
                drone_info_.at(req.assign_to_drone_id).status_msg.status != fleet_manager::DroneStatus::ACTIVE) {
                ROS_ERROR("[FleetManager] Injection failed: Drone %s is not active or does not exist.", req.assign_to_drone_id.c_str());
                res.success = false;
                return true;
            }
        }

        nlp_drone_control::Plan plan_msg;
        plan_msg.replan_mode = 0; // APPEND

        for (const auto& t : req.tasks) {
            nlp_drone_control::Action new_action = t.action_details;
            try {
                json args = json::parse(new_action.arguments_json);
                args["drone_id"] = req.assign_to_drone_id;
                new_action.arguments_json = args.dump();
                plan_msg.actions.push_back(new_action);
            } catch (const std::exception& e) {
                ROS_ERROR("[FleetManager] inject: bad arguments_json, skip one task. err=%s", e.what());
            }
        }

        ROS_INFO("[FleetManager] Injecting %zu tasks to drone %s.", plan_msg.actions.size(), req.assign_to_drone_id.c_str());
        plan_pub_.publish(plan_msg);
        res.success = true;
        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "fleet_manager_node");
    ros::NodeHandle nh;
    FleetManager manager(nh);
    ros::spin();
    return 0;
}

