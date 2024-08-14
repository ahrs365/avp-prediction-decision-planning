#include "semantic_map_manager/visualizer.h"

namespace semantic_map_manager {

void Visualizer::VisualizeGraph(const double& stamp,
                                const common::WaypointsGraph& way_graph) {
  // 清空画布
  canvas_ = cv::Mat::zeros(cv::Size(800, 600), CV_8UC3);

  // 计算所有顶点的边界框
  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();

  for (const auto& vertex : way_graph.vertices) {
    min_x = std::min(min_x, vertex->coords.first);
    max_x = std::max(max_x, vertex->coords.first);
    min_y = std::min(min_y, vertex->coords.second);
    max_y = std::max(max_y, vertex->coords.second);
  }

  // 根据边界框计算缩放比例和偏移量
  double scale_x = canvas_.cols / (max_x - min_x + 1);
  double scale_y = canvas_.rows / (max_y - min_y + 1);
  double scale = std::min(scale_x, scale_y) * 0.9;  // 留一些边界

  cv::Point2d offset(
      (canvas_.cols - (max_x - min_x) * scale) / 2 - min_x * scale,
      (canvas_.rows - (max_y - min_y) * scale) / 2 - min_y * scale);

  // 清空画布
  canvas_ = cv::Mat::zeros(cv::Size(800, 600), CV_8UC3);

  // 遍历所有边并绘制
  for (const auto& edge : way_graph.edges) {
    std::shared_ptr<common::Vertex> v1 = edge->v1;
    std::shared_ptr<common::Vertex> v2 = edge->v2;

    cv::Point2d pt1(v1->coords.first * scale + offset.x,
                    v1->coords.second * scale + offset.y);
    cv::Point2d pt2(v2->coords.first * scale + offset.x,
                    v2->coords.second * scale + offset.y);

    cv::line(canvas_, pt1, pt2, cv::Scalar(255, 255, 255),
             2);  // 白色线表示路径
  }

  // 遍历所有顶点并绘制
  for (const auto& vertex : way_graph.vertices) {
    cv::Point2d pt(vertex->coords.first * scale + offset.x,
                   vertex->coords.second * scale + offset.y);
    cv::circle(canvas_, pt, 5, cv::Scalar(0, 0, 255), -1);  // 红色圆点表示顶点
  }

  // 显示更新后的画布
  cv::imshow("Vehicle Visualization", canvas_);
  cv::waitKey(100);
}
void Visualizer::VisualizeSpots(const double& stamp,
                                const common::ParkingSpots& spots) {}
void Visualizer::VisualizeEgoVehicle(const double& stamp,

                                     const common::Vehicle& vehicle) {}
void Visualizer::VisualizeSurroundingLaneNet(
    const double& stamp, const common::LaneNet& lane_net,
    const std::vector<int>& deleted_lane_ids) {}
void Visualizer::VisualizeSurroundingVehicles(
    const double& stamp, const common::VehicleSet& vehicle_set,
    const std::vector<int>& nearby_ids) {}
void Visualizer::VisualizeObstacleMap(
    const double& stamp, const common::GridMapND<uint8_t, 2>& obstacle_map) {}

// 可视化 GridMap 的方法
void Visualizer::VisualizeGridMap(const cv::Mat& grid_map) {
  cv::imshow("Obstacle Map", grid_map);
  cv::waitKey(1);  // 等待一毫秒，确保图像刷新
}
}  // namespace semantic_map_manager
