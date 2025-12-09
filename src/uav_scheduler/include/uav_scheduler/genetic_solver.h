// genetic_solver.h
#pragma once
#include <vector>
#include <string>
#include <algorithm>
#include <random>
#include <limits>
#include <cmath>

struct GeneticSolver {
  static std::vector<int> solve(const std::vector<std::string>& drones,
                                const std::vector<std::string>& pois) {
    size_t n = drones.size();
    size_t m = pois.size();
    if (n == 0 || m == 0) return {};

    // 初始种群
    std::vector<std::vector<int>> population;
    for (int p = 0; p < 10; p++) {
      std::vector<int> indiv(n, -1);
      std::vector<int> tasks(m);
      for (size_t j = 0; j < m; j++) tasks[j] = j;
      std::shuffle(tasks.begin(), tasks.end(), std::mt19937{std::random_device{}()});
      for (size_t i = 0; i < n && i < m; i++) {
        indiv[i] = tasks[i];
      }
      population.push_back(indiv);
    }

    auto fitness = [&](const std::vector<int>& indiv) {
      double cost = 0;
      for (size_t i = 0; i < indiv.size(); i++) {
        if (indiv[i] >= 0)
          cost += std::abs((int)i - indiv[i]); // 简化距离
      }
      return -cost; // 目标是最小化 cost
    };

    // 简单遗传迭代
    for (int gen = 0; gen < 20; gen++) {
      std::sort(population.begin(), population.end(),
                [&](auto& a, auto& b){ return fitness(a) > fitness(b); });
      population.resize(5); // 保留前 5
      // 交叉
      std::vector<int> child = population[0];
      for (size_t i = 0; i < n; i++) {
        if (rand() % 2) child[i] = population[1][i];
      }
      population.push_back(child);
    }

    return population[0];
  }
};

