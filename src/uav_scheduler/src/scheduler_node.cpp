#include <ros/ros.h>
#include "uav_scheduler/ComputeAssignment.h"
#include "uav_scheduler/hungarian_solver.h"
#include "uav_scheduler/auction_solver.h"
#include "uav_scheduler/genetic_solver.h"
#include "uav_scheduler/distributed_auction_solver.h"
#include <sstream>
#include <cmath>
#include <limits>
#include <map>
#include <algorithm>

// ==== 工具函数：拼接动作计划 ====
static void buildPlan(const std::map<std::string, std::vector<std::string>>& assignments,
                      uav_scheduler::ComputeAssignment::Response& res) {
  std::stringstream plan_ss;

  for (const auto& kv : assignments) {
    const std::string& drone = kv.first;
    const auto& pois = kv.second;

    if (!pois.empty()) {
      // === assignments 列表 ===
      for (const auto& poi : pois) {
        res.assignments.push_back(drone + "->" + poi);
      }

      // === 计划动作 ===
      plan_ss << drone << ":\n";
      int step = 1;

      // 起飞一次
      plan_ss << "  " << step++ << ". Take off from the current location.\n";

      // 遍历所有 POI
      for (const auto& poi : pois) {
        plan_ss << "  " << step++ << ". Go to the best approach point for " << poi << ".\n";
        plan_ss << "  " << step++ << ". Inspect " << poi << ".\n";
      }

      // 最后返航+降落
      plan_ss << "  " << step++ << ". Return to the launch location.\n";
      plan_ss << "  " << step++ << ". Land.\n";

    } else {
      // 没任务 → IDLE
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


// ==== 多轮 Hungarian ====
static std::map<std::string, std::vector<std::string>>
multiRoundHungarian(const std::vector<std::string>& drones,
                    const std::vector<std::string>& pois) {
  std::map<std::string, std::vector<std::string>> result;
  std::vector<std::string> remaining_pois = pois;

  // 初始每个无人机分配空队列
  for (const auto& d : drones) {
    result[d] = {};
  }

  while (!remaining_pois.empty()) {
    size_t m = drones.size();
    size_t n = remaining_pois.size();

    // 构造当前 cost matrix (示例用 i,j 索引差表示距离，可改为真实距离)
    std::vector<std::vector<double>> cost(m, std::vector<double>(n, 0));
    for (size_t i = 0; i < m; i++) {
      for (size_t j = 0; j < n; j++) {
        cost[i][j] = std::abs((int)i - (int)j);
      }
    }

    // 求解 Hungarian
    std::vector<int> assignment = HungarianSolver::solve(cost);

    // 收集结果
    std::vector<std::string> assigned;
    for (size_t i = 0; i < assignment.size(); i++) {
      int j = assignment[i];
      if (j >= 0 && j < (int)n) {
        std::string poi = remaining_pois[j];
        result[drones[i]].push_back(poi);
        ROS_INFO_STREAM("[ROUND MATCH] " << drones[i] << " -> " << poi);
        assigned.push_back(poi);
      }
    }

    // 剔除已分配的 POI
    remaining_pois.erase(
      std::remove_if(remaining_pois.begin(), remaining_pois.end(),
                     [&](const std::string& poi) {
                       return std::find(assigned.begin(), assigned.end(), poi) != assigned.end();
                     }),
      remaining_pois.end()
    );
  }

  return result;
}

// ==== 回调 ====
bool compute(uav_scheduler::ComputeAssignment::Request& req,
             uav_scheduler::ComputeAssignment::Response& res) {
  if (req.drones.empty() || req.pois.empty()) {
    ROS_ERROR("Scheduler request missing drones or POIs!");
    return false;
  }

  std::map<std::string, std::vector<std::string>> assignment_map;

  if (req.algorithm == "Hungarian Algorithm") {
    ROS_INFO("Running Multi-Round Hungarian Scheduler...");
    assignment_map = multiRoundHungarian(req.drones, req.pois);

  } else if (req.algorithm == "Auction Algorithm") {
    ROS_INFO("Running Auction Scheduler...");
    // TODO: 多轮 auction 分配
    for (size_t i = 0; i < req.drones.size(); i++) {
      assignment_map[req.drones[i]] = {req.pois[i % req.pois.size()]};
    }

  } else if (req.algorithm == "Genetic Algorithm") {
    ROS_INFO("Running Genetic Scheduler...");
    for (size_t i = 0; i < req.drones.size(); i++) {
      assignment_map[req.drones[i]] = {req.pois[i % req.pois.size()]};
    }

  } else if (req.algorithm == "Distributed Auction Algorithm") {
    ROS_INFO("Running Distributed Auction Scheduler...");
    for (size_t i = 0; i < req.drones.size(); i++) {
      assignment_map[req.drones[i]] = {req.pois[i % req.pois.size()]};
    }

  } else {
    ROS_ERROR_STREAM("Unknown algorithm: " << req.algorithm);
    return false;
  }

  buildPlan(assignment_map, res);
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "scheduler_node");
  ros::NodeHandle nh;
  ros::ServiceServer srv = nh.advertiseService("compute_assignment", compute);
  ROS_INFO("Scheduler service ready.");
  ros::spin();
  return 0;
}

