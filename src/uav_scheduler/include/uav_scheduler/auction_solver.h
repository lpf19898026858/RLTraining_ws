// auction_solver.h
#pragma once
#include <vector>
#include <string>
#include <limits>
#include <cmath>

struct AuctionSolver {
  static std::vector<int> solve(const std::vector<std::string>& drones,
                                const std::vector<std::string>& pois) {
    size_t n = drones.size();
    size_t m = pois.size();
    std::vector<int> assignment(n, -1);
    std::vector<bool> taken(m, false);

    for (size_t i = 0; i < n; i++) {
      double best_score = -1e9;
      int best_j = -1;
      for (size_t j = 0; j < m; j++) {
        if (!taken[j]) {
          double score = -(std::abs((int)i - (int)j)); // 简化: 距离越近，分数越高
          if (score > best_score) {
            best_score = score;
            best_j = j;
          }
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

