#include "uav_scheduler/hungarian_solver.h"
#include <limits>
#include <cmath>

std::vector<int> HungarianSolver::solve(const std::vector<std::vector<double>>& cost) {
  size_t n = cost.size();
  size_t m = cost[0].size();
  std::vector<int> assignment(n, -1);
  std::vector<bool> used(m, false);

  for (size_t i = 0; i < n; i++) {
    double best = std::numeric_limits<double>::max();
    int best_j = -1;
    for (size_t j = 0; j < m; j++) {
      if (!used[j] && cost[i][j] < best) {
        best = cost[i][j];
        best_j = j;
      }
    }
    if (best_j >= 0) {
      assignment[i] = best_j;
      used[best_j] = true;
    }
  }
  return assignment;
}

