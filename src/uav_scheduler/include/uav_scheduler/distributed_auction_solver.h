#pragma once
#include <vector>
#include <limits>
#include <cmath>

struct DistributedAuctionSolver {
  static std::vector<int> solve(const std::vector<std::vector<double>>& cost_matrix) {
    size_t n = cost_matrix.size();               // drones 数量
    size_t m = n > 0 ? cost_matrix[0].size() : 0; // POIs 数量
    std::vector<int> assignment(n, -1);
    std::vector<bool> taken(m, false);

    // 模拟分布式：每个无人机独立选择最近的 POI（按 cost）
    for (size_t i = 0; i < n; ++i) {
      double best_cost = std::numeric_limits<double>::max();
      int best_j = -1;
      for (size_t j = 0; j < m; ++j) {
        double cost = cost_matrix[i][j];
        if (cost < best_cost && !taken[j]) {
          best_cost = cost;
          best_j = j;
        }
      }
      if (best_j >= 0) {
        assignment[i] = best_j;
        taken[best_j] = true;
      }
    }
    return assignment;
  }
};

