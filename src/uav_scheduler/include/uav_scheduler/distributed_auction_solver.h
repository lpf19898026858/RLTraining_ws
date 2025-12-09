// distributed_auction_solver.h
#pragma once
#include <vector>
#include <string>
#include <limits>
#include <cmath>

struct DistributedAuctionSolver {
  static std::vector<int> solve(const std::vector<std::string>& drones,
                                const std::vector<std::string>& pois) {
    size_t n = drones.size();
    size_t m = pois.size();
    std::vector<int> assignment(n, -1);
    std::vector<bool> taken(m, false);

    // 模拟分布式：每个无人机独立选择最近的 POI
    for (size_t i = 0; i < n; i++) {
      double best_cost = std::numeric_limits<double>::max();
      int best_j = -1;
      for (size_t j = 0; j < m; j++) {
        double cost = std::abs((int)i - (int)j); // 简化距离
        if (cost < best_cost) {
          best_cost = cost;
          best_j = j;
        }
      }
      if (best_j >= 0 && !taken[best_j]) {
        assignment[i] = best_j;
        taken[best_j] = true;
      }
    }
    return assignment;
  }
};

