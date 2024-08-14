#pragma once
#include <cmath>
#include <iterator>
#include <memory>  // 包含智能指针库
#include <queue>
#include <set>
#include <vector>

#include "common/lane/graph.h"

namespace common {

// AStarGraph 类：用于存储 A* 结果路径的图
class AStarGraph : public WaypointsGraph {
 public:
  AStarGraph(const std::vector<std::shared_ptr<Edge>>& path) {
    edges = path;

    if (!path.empty()) {
      vertices.push_back(path.front()->v1);
      for (const auto& e : path) {
        vertices.push_back(e->v2);
      }
    }
  }

  // 计算路径的总成本
  double path_cost() const {
    double cost = 0;
    for (const auto& e : edges) {
      cost += e->cost;
    }
    return cost;
  }

  // 绘制图形（可根据实际需求实现）
  void plot() const {
    std::cout << "Plotting A* result..." << std::endl;
    // 实现绘图逻辑（例如使用 matplotlib 或其他图形库）
  }

  // 计算参考路径（可根据实际需求实现）
  std::vector<std::pair<double, double>> compute_ref_path(
      double offset = 0.0) const {
    std::vector<std::pair<double, double>> route;
    // 收集 A* 解中的 x, y
    for (const auto& v : vertices) {
      route.push_back({v->coords.first, v->coords.second});
    }
    return route;

    // 计算样条线或路径偏移逻辑...
  }
};

// AStarPlanner 类：使用 A* 算法在图上规划最短路径
class AStarPlanner {
 public:
  AStarPlanner(const std::shared_ptr<Vertex>& v_start,
               const std::shared_ptr<Vertex>& v_goal)
      : v_start(v_start), v_goal(v_goal), counter(0) {
    fringe.emplace(
        0, counter++,
        std::make_tuple(v_start, std::vector<std::shared_ptr<Edge>>{}, 0.0));
  }

  AStarGraph solve() {
    while (!fringe.empty()) {
      auto current = fringe.top();
      fringe.pop();

      auto v = std::get<0>(std::get<2>(current));
      auto path = std::get<1>(std::get<2>(current));
      double cost = std::get<2>(std::get<2>(current));

      if (v == v_goal) {
        return AStarGraph(path);
      }

      if (closed.find(v) == closed.end()) {
        closed.insert(v);

        for (size_t i = 0; i < v->children.size(); ++i) {
          auto child = v->children[i];
          auto edge = v->edges[i];

          double new_cost = cost + edge->cost;
          double a_star_cost = new_cost + child->dist(v_goal);

          auto new_path = path;
          new_path.push_back(edge);

          fringe.emplace(a_star_cost, counter++,
                         std::make_tuple(child, new_path, new_cost));
        }
      }
    }

    throw std::runtime_error("Path is not found");
  }

 private:
  std::shared_ptr<Vertex> v_start;
  std::shared_ptr<Vertex> v_goal;
  std::priority_queue<
      std::tuple<double, int,
                 std::tuple<std::shared_ptr<Vertex>,
                            std::vector<std::shared_ptr<Edge>>, double>>,
      std::vector<
          std::tuple<double, int,
                     std::tuple<std::shared_ptr<Vertex>,
                                std::vector<std::shared_ptr<Edge>>, double>>>,
      std::greater<>>
      fringe;                                // 优先队列用于 A* 搜索
  std::set<std::shared_ptr<Vertex>> closed;  // 已访问的节点
  int counter;  // 计数器用于防止优先队列中具有相同成本的节点混淆
};

}  // namespace common
