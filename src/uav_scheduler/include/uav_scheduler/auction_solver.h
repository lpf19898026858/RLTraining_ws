// auction_solver.h
#pragma once
#include <vector>
#include <string>
#include <limits>
#include <cmath>

struct AuctionSolver {
  static std::vector<int> solve(const std::vector<std::vector<double>>& cost_matrix) {
  size_t m = cost_matrix.size();
  size_t n = m > 0 ? cost_matrix[0].size() : 0;

  std::vector<int> assignment(m, -1);

  // 示例：贪心式分配（可以替换成真正的拍卖算法）
  for (size_t i = 0; i < m; ++i) {
    double best = 1e9;
    int best_j = -1;
    for (size_t j = 0; j < n; ++j) {
      if (cost_matrix[i][j] < best) {
        best = cost_matrix[i][j];
        best_j = j;
      }
    }
    assignment[i] = best_j;
  }
  return assignment;
  }
};

