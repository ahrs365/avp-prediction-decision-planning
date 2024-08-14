#include "semantic_map_manager/visualizer.h"

namespace semantic_map_manager {

void Visualizer::VisualizeGraph(const double& stamp,
                                const common::WaypointsGraph& way_graph) {}
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
