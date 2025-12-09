#include <ros/ros.h>
#include "uav_scheduler/ComputeAssignment.h"
#include "poi_state_server/GetPOIInfo.h"
#include "geometry_msgs/Point.h"
#include "uav_scheduler/hungarian_solver.h"
#include "uav_scheduler/auction_solver.h"
#include "uav_scheduler/genetic_solver.h"
#include "uav_scheduler/distributed_auction_solver.h"
#include <sstream>
#include <cmath>
#include <limits>
#include <map>
#include <algorithm>
#include <unordered_set>

//====================== 小工具：清洗/去重 ======================

static inline std::string trim(const std::string& s) {
  size_t a = s.find_first_not_of(" \t\r\n");
  if (a == std::string::npos) return "";
  size_t b = s.find_last_not_of(" \t\r\n");
  return s.substr(a, b - a + 1);
}

static inline std::string rstrip_punct(const std::string& s) {
  size_t end = s.find_last_not_of(" \t\r\n.,;:、");
  if (end == std::string::npos) return "";
  return s.substr(0, end + 1);
}

static std::vector<std::string> sanitizePOIs(const std::vector<std::string>& in) {
  std::vector<std::string> out;
  out.reserve(in.size());
  std::unordered_set<std::string> seen;
  for (auto s : in) {
    s = rstrip_punct(trim(s));
    if (s.empty()) continue;
    if (seen.insert(s).second) out.push_back(s);
  }
  return out;
}

//====================== 构造距离代价矩阵（使用缓存坐标 + 最佳观察点 + 方阵补齐） ======================

static std::vector<std::vector<double>> buildDistanceCostMatrix(
    const std::vector<std::string>& drones,
    const std::vector<std::string>& pois,
    const std::map<std::string, geometry_msgs::Point>& drone_positions,
    const std::map<std::string, geometry_msgs::Point>& poi_positions)
{
  size_t m = drones.size();
  size_t n = pois.size();
  size_t S = std::max(m, n);
  const double BIG = 1e6; // dummy slot 代价

  std::vector<std::vector<double>> cost(S, std::vector<double>(S, BIG));

  for (size_t i = 0; i < m; ++i) {
    for (size_t j = 0; j < n; ++j) {
      const auto& dp = drone_positions.at(drones[i]);
      const auto& pp = poi_positions.at(pois[j]);
      double dx = dp.x - pp.x;
      double dy = dp.y - pp.y;
      double dz = dp.z - pp.z;
      cost[i][j] = std::sqrt(dx * dx + dy * dy + dz * dz);
    }
  }
  return cost;
}

//====================== 从 poi_state_server 缓存一次坐标（含最佳观察点） ======================

static void fetchPositions(
    const std::vector<std::string>& drones,
    const std::vector<std::string>& pois,
    ros::ServiceClient& poi_client,
    std::map<std::string, geometry_msgs::Point>& drone_positions,
    std::map<std::string, geometry_msgs::Point>& poi_positions)
{
  // --- 无人机 ---
  for (const auto& d : drones) {
    poi_state_server::GetPOIInfo srv;
    srv.request.name = d;
    if (poi_client.call(srv) && srv.response.success) {
      drone_positions[d] = srv.response.info.position;
      ROS_INFO_STREAM("[UAV] " << d << " -> ("
                      << srv.response.info.position.x << ", "
                      << srv.response.info.position.y << ", "
                      << srv.response.info.position.z << ")");
    } else {
      ROS_WARN_STREAM("[UAV] " << d << " 获取失败，默认 (0,0,0)");
      drone_positions[d] = geometry_msgs::Point();
    }
  }

  // --- POI ---
  for (const auto& p : pois) {
    poi_state_server::GetPOIInfo srv;
    srv.request.name = p;
    if (poi_client.call(srv) && srv.response.success) {
      geometry_msgs::Point pos;
      // ✅ 若有候选点则取最佳观察点
      if (!srv.response.info.candidate_points.empty()) {
        pos = srv.response.info.candidate_points[0].point;
      } else {
        pos = srv.response.info.position;
      }
      poi_positions[p] = pos;
      ROS_INFO_STREAM("[POI] " << p << " -> ("
                      << pos.x << ", " << pos.y << ", " << pos.z << ")");
    } else {
    ROS_WARN_STREAM("[POI] " << p << " 不存在于环境中，将跳过此目标。");
    continue;  // ✅ 跳过该POI，而不是赋(0,0,0)
    }
  }
}

//====================== 计划文本拼接（保持不变） ======================

static void buildPlan(const std::map<std::string, std::vector<std::string>>& assignments,
                      uav_scheduler::ComputeAssignment::Response& res) {
  std::stringstream plan_ss;

  for (const auto& kv : assignments) {
    const std::string& drone = kv.first;
    const auto& pois = kv.second;

    if (!pois.empty()) {
      for (const auto& poi : pois)
        res.assignments.push_back(drone + "->" + poi);

      plan_ss << drone << ":\n";
      int step = 1;
      plan_ss << "  " << step++ << ". Take off from the current location.\n";
      for (const auto& poi : pois) {
        plan_ss << "  " << step++ << ". Go to the best approach point for " << poi << ".\n";
        plan_ss << "  " << step++ << ". Inspect " << poi << ".\n";
      }
      plan_ss << "  " << step++ << ". Return to the launch location.\n";
      plan_ss << "  " << step++ << ". Land.\n";
    } else {
      res.assignments.push_back(drone + "->IDLE");
      plan_ss << drone << ":\n";
      plan_ss << "  1. Take off from the current location.\n";
      plan_ss << "  2. Wait 10 seconds.\n";
      plan_ss << "  3. Return to the launch location.\n";
      plan_ss << "  4. Land.\n";
    }
  }
  res.plan_text = plan_ss.str();
}

//====================== 多轮求解器封装（基于缓存距离矩阵） ======================

using SolveFn = std::vector<int>(*)(const std::vector<std::vector<double>>&);

static std::map<std::string, std::vector<std::string>>
multiRoundSolve(const std::vector<std::string>& drones,
                const std::vector<std::string>& pois,
                SolveFn solver_fn,
                const char* solver_name,
                const std::map<std::string, geometry_msgs::Point>& drone_positions,
                const std::map<std::string, geometry_msgs::Point>& poi_positions)
{
  std::map<std::string, std::vector<std::string>> result;
  for (const auto& d : drones) result[d] = {};

  std::vector<std::string> remaining = pois;
  const size_t m = drones.size();

  while (!remaining.empty()) {
    size_t n = remaining.size();
    auto cost = buildDistanceCostMatrix(drones, remaining, drone_positions, poi_positions);

    std::vector<int> assign;
    try {
      assign = solver_fn(cost);
    } catch (const std::exception& e) {
      ROS_ERROR_STREAM("[" << solver_name << "] Solver exception: " << e.what());
      break;
    }

    std::vector<std::string> assigned;
    for (size_t i = 0; i < m && i < assign.size(); ++i) {
      int j = assign[i];
      if (j >= 0 && j < (int)n) {
        const std::string& poi = remaining[j];
        result.at(drones[i]).push_back(poi);
        assigned.push_back(poi);
        ROS_INFO_STREAM("[MATCH:" << solver_name << "] " << drones[i]
                        << " -> " << poi << " (dist=" << cost[i][j] << ")");
      }
    }

    if (assigned.empty()) break;

    remaining.erase(std::remove_if(remaining.begin(), remaining.end(),
      [&](const std::string& p){
        return std::find(assigned.begin(), assigned.end(), p) != assigned.end();
      }), remaining.end());
  }

  return result;
}

//====================== Service 回调 ======================

bool compute(uav_scheduler::ComputeAssignment::Request& req,
             uav_scheduler::ComputeAssignment::Response& res)
{
  if (req.drones.empty() || req.pois.empty()) {
    ROS_ERROR("Scheduler request missing drones or POIs!");
    return false;
  }

  std::vector<std::string> clean_pois = sanitizePOIs(req.pois);
  if (clean_pois.empty()) {
    ROS_ERROR("All POIs were empty after sanitization.");
    return false;
  }

  ros::NodeHandle nh;
  ros::ServiceClient poi_client =
      nh.serviceClient<poi_state_server::GetPOIInfo>("/poi_state_server/get_poi_info");

  // ✅ 一次性缓存所有坐标
  std::map<std::string, geometry_msgs::Point> uav_positions, poi_positions;
  fetchPositions(req.drones, clean_pois, poi_client, uav_positions, poi_positions);

  std::map<std::string, std::vector<std::string>> assignment_map;

  if (req.algorithm == "Hungarian Algorithm") {
    assignment_map = multiRoundSolve(req.drones, clean_pois, &HungarianSolver::solve,
                                     "Hungarian", uav_positions, poi_positions);
  } else if (req.algorithm == "Auction Algorithm") {
    assignment_map = multiRoundSolve(req.drones, clean_pois, &AuctionSolver::solve,
                                     "Auction", uav_positions, poi_positions);
  } else if (req.algorithm == "Genetic Algorithm") {
    assignment_map = multiRoundSolve(req.drones, clean_pois, &GeneticSolver::solve,
                                     "Genetic", uav_positions, poi_positions);
  } else if (req.algorithm == "Distributed Auction Algorithm") {
    assignment_map = multiRoundSolve(req.drones, clean_pois, &DistributedAuctionSolver::solve,
                                     "DistributedAuction", uav_positions, poi_positions);
  } else {
    ROS_ERROR_STREAM("Unknown algorithm: " << req.algorithm);
    return false;
  }

  buildPlan(assignment_map, res);
  return true;
}

//====================== Main ======================

int main(int argc, char** argv) {
  ros::init(argc, argv, "scheduler_node");
  ros::NodeHandle nh;
  ros::ServiceServer srv = nh.advertiseService("compute_assignment", compute);
  ROS_INFO("Scheduler service ready.");
  ros::spin();
  return 0;
}

