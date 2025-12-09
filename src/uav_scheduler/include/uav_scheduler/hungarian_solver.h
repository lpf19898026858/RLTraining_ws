#pragma once
#include <vector>
#include <string>
#include <geometry_msgs/Point.h>

class HungarianSolver {
public:
  static std::vector<int> solve(const std::vector<std::vector<double>>& cost);
};

