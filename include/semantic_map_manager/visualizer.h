#ifndef VISUALIZER_H
#define VISUALIZER_H
#include <assert.h>

#include <iostream>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <vector>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/lane/graph.h"
#include "common/state/state.h"
#include "common/visualization/common_visualization_util.h"

namespace semantic_map_manager {
class Visualizer {
 public:
  // 获取单例实例
  static Visualizer &GetInstance() {
    static Visualizer instance;
    return instance;
  }
  // 禁止复制构造和赋值操作
  Visualizer(const Visualizer &) = delete;
  Visualizer &operator=(const Visualizer &) = delete;

  void VisualizeData(const double &stamp,
                     const common::WaypointsGraph &way_graph,
                     const common::ParkingSpots &spots,
                     const common::Vehicle &vehicle,
                     const common::LaneNet &lane_net,
                     const common::ObstacleSet &obstacle_set,
                     const common::VehicleSet &vehicle_set,
                     const common::GridMapND<uint8_t, 2> &obstacle_map,
                     const std::set<std::array<decimal_t, 2>> &grid_map);

  void VisualizeGraph(const double &stamp,
                      const common::WaypointsGraph &way_graph);
  void VisualizeSpots(const double &stamp, const common::ParkingSpots &spots);
  void VisualizeEgoVehicle(const double &stamp, const common::Vehicle &vehicle);
  void VisualizeSurroundingLaneNet(const double &stamp,
                                   const common::LaneNet &lane_net,
                                   const std::vector<int> &deleted_lane_ids);

  void VisualizeSurroundingVehicles(const double &stamp,
                                    const common::VehicleSet &vehicle_set,
                                    const std::vector<int> &nearby_ids);

  void VisualizeObstacleMap(const double &stamp,
                            const common::GridMapND<uint8_t, 2> &obstacle_map);

  void VisualizeStaticObstacle(const double &stamp,
                               const common::ObstacleSet &obstacle_set);

  void VisualizeGridMap(const std::set<std::array<decimal_t, 2>> &grid_map);

 private:
  int node_id_;
  int width_ = 1600;
  int height_ = 1200;
  double map_width_ = 140;
  double map_height_ = 80;
  double scale_;
  cv::Point2d offset_;

 private:
  cv::Mat canvas_;    // 画布，用于绘制车辆、障碍物等
  std::mutex mutex_;  // 互斥锁，保证线程安全

  // 构造函数
  Visualizer() {
    // 初始化画布大小和颜色
    canvas_ = cv::Mat::zeros(cv::Size(width_, height_), CV_8UC3);

    // 创建并显示窗口
    cv::namedWindow("Vehicle Visualization", cv::WINDOW_AUTOSIZE);
  }

  // 析构函数
  ~Visualizer() {
    // 关闭窗口，释放资源
    cv::destroyWindow("Vehicle Visualization");
  }

};  // class Visualizer
}  // namespace semantic_map_manager

#endif