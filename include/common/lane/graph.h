#pragma once
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>  // 包含智能指针库
#include <queue>
#include <set>
#include <vector>

namespace common {
class Edge;
class Vertex;

// 顶点类
class Vertex {
 public:
  std::pair<double, double> coords;               // 顶点的坐标 (x, y)
  std::vector<std::shared_ptr<Vertex>> children;  // 连接的子节点
  std::vector<std::shared_ptr<Edge>> edges;  // 连接的边（存储的是边的长度）

  Vertex(double x, double y) : coords(x, y) {}

  // 计算当前顶点与另一个顶点的距离
  double dist(const std::shared_ptr<Vertex>& v) const {
    return std::sqrt(std::pow(coords.first - v->coords.first, 2) +
                     std::pow(coords.second - v->coords.second, 2));
  }

  // 添加一个子顶点以及对应的边
  void add_child(const std::shared_ptr<Vertex>& v,
                 const std::shared_ptr<Edge>& edge) {
    children.push_back(v);
    edges.push_back(edge);
  }

  bool operator==(const Vertex& v) const { return coords == v.coords; }
};

// 边类
class Edge {
 public:
  std::shared_ptr<Vertex> v1;  // 边的起点
  std::shared_ptr<Vertex> v2;  // 边的终点
  double cost;                 // 边的权重（长度）

  Edge(const std::shared_ptr<Vertex>& vertex1,
       const std::shared_ptr<Vertex>& vertex2, double c)
      : v1(vertex1), v2(vertex2), cost(c) {}
};

// 航路点图类
class WaypointsGraph {
 public:
  std::vector<std::shared_ptr<Vertex>> vertices;  // 顶点集合
  std::vector<std::shared_ptr<Edge>> edges;       // 边集合

  // 向图中添加一系列航路点，并按顺序连接
  void add_waypoint_list(
      const std::vector<std::pair<double, double>>& waypoint_list) {
    std::vector<std::shared_ptr<Vertex>> v_list;

    // 创建所有顶点
    for (const auto& waypoint : waypoint_list) {
      v_list.push_back(
          std::make_shared<Vertex>(waypoint.first, waypoint.second));
    }

    // 创建并添加边
    for (size_t i = 0; i < v_list.size() - 1; ++i) {
      double dist = v_list[i]->dist(v_list[i + 1]);

      // 创建正向边
      auto edge_fwd = std::make_shared<Edge>(v_list[i], v_list[i + 1], dist);
      v_list[i]->add_child(v_list[i + 1], edge_fwd);
      edges.push_back(edge_fwd);

      // 创建反向边
      auto edge_rev = std::make_shared<Edge>(v_list[i + 1], v_list[i], dist);
      v_list[i + 1]->add_child(v_list[i], edge_rev);
      edges.push_back(edge_rev);
    }

    // 添加顶点到图中
    vertices.insert(vertices.end(), v_list.begin(), v_list.end());
  }

  // 查找与给定坐标最接近的顶点
  std::shared_ptr<Vertex> search(const std::pair<double, double>& coords) {
    double min_dist = std::numeric_limits<double>::infinity();
    std::shared_ptr<Vertex> closest_vertex = nullptr;

    for (auto& v : vertices) {
      double dist = std::sqrt(std::pow(coords.first - v->coords.first, 2) +
                              std::pow(coords.second - v->coords.second, 2));
      if (dist < min_dist) {
        min_dist = dist;
        closest_vertex = v;
      }
    }

    return closest_vertex;
  }

  // 连接两个顶点
  void connect(const std::pair<double, double>& coords_v1,
               const std::pair<double, double>& coords_v2) {
    auto v1 = search(coords_v1);
    auto v2 = search(coords_v2);

    if (v1 == v2) {
      std::cout << "The specified locations are too close to each other. No "
                   "new connection will be created."
                << std::endl;
      return;
    }

    double dist = v1->dist(v2);

    // 创建正向边
    auto edge_fwd = std::make_shared<Edge>(v1, v2, dist);
    v1->add_child(v2, edge_fwd);
    edges.push_back(edge_fwd);

    // 创建反向边
    auto edge_rev = std::make_shared<Edge>(v2, v1, dist);
    v2->add_child(v1, edge_rev);
    edges.push_back(edge_rev);
  }

  // 添加一个顶点到图中
  void add_vertex(const std::shared_ptr<Vertex>& vertex) {
    vertices.push_back(vertex);
  }

  // 添加一条边到图中
  void add_edge(const std::shared_ptr<Vertex>& vertex1,
                const std::shared_ptr<Vertex>& vertex2) {
    double dist = vertex1->dist(vertex2);
    auto edge = std::make_shared<Edge>(vertex1, vertex2, dist);
    edges.push_back(edge);
    vertex1->add_child(vertex2, edge);
    vertex2->add_child(vertex1, edge);
  }

  void print_graph() const {
    for (const auto& vertex : vertices) {
      std::cout << "Vertex: (" << vertex->coords.first << ", "
                << vertex->coords.second << ")" << std::endl;
      for (size_t i = 0; i < vertex->children.size(); ++i) {
        std::cout << "  -> Connected to: (" << vertex->children[i]->coords.first
                  << ", " << vertex->children[i]->coords.second
                  << ") with edge length: " << vertex->edges[i]->cost
                  << std::endl;
      }
    }
  }
};

}  // namespace common
