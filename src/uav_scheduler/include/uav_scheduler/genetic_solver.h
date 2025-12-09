#pragma once
#include <vector>
#include <algorithm>
#include <random>
#include <limits>
#include <cmath>

struct GeneticSolver {
  static std::vector<int> solve(const std::vector<std::vector<double>>& cost_matrix) {
    size_t n = cost_matrix.size();               // drones 数量
    size_t m = n > 0 ? cost_matrix[0].size() : 0; // POIs 数量
    if (n == 0 || m == 0) return {};

    // 初始化种群
    const int POP_SIZE = 10;
    std::vector<std::vector<int>> population;
    for (int p = 0; p < POP_SIZE; ++p) {
      std::vector<int> indiv(n, -1);
      std::vector<int> tasks(m);
      for (size_t j = 0; j < m; ++j) tasks[j] = j;
      std::shuffle(tasks.begin(), tasks.end(), std::mt19937{std::random_device{}()});
      for (size_t i = 0; i < n && i < m; ++i) indiv[i] = tasks[i];
      population.push_back(indiv);
    }

    // 计算适应度（越小越好 → 转为负号）
    auto fitness = [&](const std::vector<int>& indiv) {
      double total_cost = 0;
      for (size_t i = 0; i < indiv.size(); ++i) {
        int j = indiv[i];
        if (j >= 0 && j < (int)m)
          total_cost += cost_matrix[i][j];
        else
          total_cost += 1e6; // 无效项惩罚
      }
      return -total_cost;
    };

    // 遗传迭代
    const int GENERATIONS = 30;
    for (int gen = 0; gen < GENERATIONS; ++gen) {
      std::sort(population.begin(), population.end(),
                [&](const auto& a, const auto& b) { return fitness(a) > fitness(b); });
      population.resize(5);  // 保留前 5

      // 简单交叉 + 变异
      std::vector<int> child = population[0];
      for (size_t i = 0; i < n; ++i) {
        if (rand() % 2) child[i] = population[1][i];
        if (rand() % 10 == 0) child[i] = rand() % m; // 轻微变异
      }
      population.push_back(child);
    }

    // 取最优个体
    return population[0];
  }
};

