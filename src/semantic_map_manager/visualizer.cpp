#include "semantic_map_manager/visualizer.h"

namespace semantic_map_manager {
void Visualizer::VisualizeData(
    const double& stamp, const common::WaypointsGraph& way_graph,
    const common::ParkingSpots& spots, const common::Vehicle& vehicle,
    const common::LaneNet& lane_net, const common::ObstacleSet& obstacle_set,
    const common::VehicleSet& vehicle_set,
    const common::GridMapND<uint8_t, 2>& obstacle_map,
    const std::set<std::array<decimal_t, 2>>& grid_map) {
  // 清空画布
  canvas_ =
      cv::Mat(cv::Size(width_, height_), CV_8UC3, cv::Scalar(255, 255, 255));

  double min_x = 0;
  double min_y = 0;
  double max_x = map_width_;
  double max_y = map_height_;

  // 根据边界框计算缩放比例和偏移量
  double scale_x = canvas_.cols / (max_x - min_x + 1);
  double scale_y = canvas_.rows / (max_y - min_y + 1);
  scale_ = std::min(scale_x, scale_y) * 0.9;  // 留一些边界

  offset_ = cv::Point2d(
      (canvas_.cols - (max_x - min_x) * scale_) / 2 - min_x * scale_,
      (canvas_.rows - (max_y - min_y) * scale_) / 2 - min_y * scale_);

  VisualizeGraph(stamp, way_graph);
  VisualizeSpots(stamp, spots);
  VisualizeEgoVehicle(stamp, vehicle);
  VisualizeSurroundingLaneNet(stamp, lane_net, {});
  VisualizeStaticObstacle(stamp, obstacle_set);
  VisualizeSurroundingVehicles(stamp, vehicle_set, {});
  VisualizeObstacleMap(stamp, obstacle_map);
  VisualizeGridMap(grid_map);
  cv::imshow("Vehicle Visualization", canvas_);
  cv::waitKey(1);
}
void Visualizer::VisualizeGraph(const double& stamp,

                                const common::WaypointsGraph& way_graph) {
  // 遍历所有边并绘制
  for (const auto& edge : way_graph.edges) {
    std::shared_ptr<common::Vertex> v1 = edge->v1;
    std::shared_ptr<common::Vertex> v2 = edge->v2;

    cv::Point2d pt1(v1->coords.first * scale_ + offset_.x,
                    v1->coords.second * scale_ + offset_.y);
    cv::Point2d pt2(v2->coords.first * scale_ + offset_.x,
                    v2->coords.second * scale_ + offset_.y);

    cv::line(canvas_, pt1, pt2, cv::Scalar(255, 0, 255),
             2);  // 白色线表示路径
  }

  // 遍历所有顶点并绘制
  for (const auto& vertex : way_graph.vertices) {
    cv::Point2d pt(vertex->coords.first * scale_ + offset_.x,
                   vertex->coords.second * scale_ + offset_.y);
    cv::circle(canvas_, pt, 5, cv::Scalar(0, 0, 255), -1);  // 红色圆点表示顶点
  }
}
void Visualizer::VisualizeSpots(const double& stamp,
                                const common::ParkingSpots& spots) {
  // 遍历所有停车位并绘制
  for (const auto& spot_pair : spots.spot_polygon) {
    const auto& spot = spot_pair.second;

    // 绘制停车位的多边形
    const auto& polygon = spot.polygon;
    std::vector<cv::Point2d> polygon_points;
    for (const auto& point : polygon.points) {
      cv::Point2d pt(point.x * scale_ + offset_.x,
                     point.y * scale_ + offset_.y);
      polygon_points.push_back(pt);
    }

    // 如果多边形有足够的点，则绘制它
    if (polygon_points.size() >= 3) {
      for (size_t i = 0; i < polygon_points.size(); ++i) {
        const auto& pt1 = polygon_points[i];
        const auto& pt2 = polygon_points[(i + 1) % polygon_points.size()];
        cv::line(canvas_, pt1, pt2, cv::Scalar(0, 0, 0),
                 2);  // 绿色线表示停车位边界
      }
    }

    // 在多边形的中心位置绘制停车位ID
    cv::Point2d center(0, 0);
    for (const auto& pt : polygon_points) {
      center += pt;
    }
    center.x /= polygon_points.size();
    center.y /= polygon_points.size();

    cv::putText(canvas_, std::to_string(spot.id), center,
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
  }
}
void Visualizer::VisualizeEgoVehicle(const double& stamp,

                                     const common::Vehicle& vehicle) {
  // 获取车辆的 OBB (Oriented BoundingBox)
  common::OrientedBoundingBox2D obb = vehicle.RetOrientedBoundingBox();

  // 获取车辆的四个顶点
  vec_E<Vec2f> vertices;
  common::SemanticsUtils::GetVehicleVertices(vehicle.param(), vehicle.state(),
                                             &vertices);

  // 将顶点转换为 cv::Point2i，并按照 scale 和 offset 进行缩放和平移
  std::vector<cv::Point> polygon_points;
  for (const auto& vertex : vertices) {
    polygon_points.push_back(
        cv::Point(static_cast<int>(vertex.x() * scale_ + offset_.x),
                  static_cast<int>(vertex.y() * scale_ + offset_.y)));
  }

  // 创建一个与画布相同大小的临时图层
  cv::Mat overlay;
  canvas_.copyTo(overlay);

  // 在临时图层上填充车辆轮廓 (半透明蓝色)
  cv::fillPoly(overlay, std::vector<std::vector<cv::Point>>{polygon_points},
               cv::Scalar(0, 0, 255, 100));  // 100 表示透明度

  // 将临时图层与原始画布混合，生成半透明效果
  double alpha = 0.4;  // 控制透明度
  cv::addWeighted(overlay, alpha, canvas_, 1 - alpha, 0, canvas_);

  // 绘制车辆轮廓 (多边形)
  cv::polylines(canvas_, polygon_points, true, cv::Scalar(0, 0, 255),
                2);  // 绿色线表示车辆

  // 绘制速度箭头
  const auto& state = vehicle.state();
  double velocity = state.velocity;  // 获取车辆的速度大小
  double angle = state.angle;        // 获取车辆的朝向

  // // 判断车辆是否在倒车 (根据速度和角度关系)
  // if (state.is_reverse) {  // 假设 `state.is_reverse` 标识车辆是否在倒车
  //   velocity = -velocity;  // 如果在倒车，反向调整速度的箭头方向
  // }

  // 计算速度箭头的起点和终点
  cv::Point arrow_start(static_cast<int>(obb.x * scale_ + offset_.x),
                        static_cast<int>(obb.y * scale_ + offset_.y));
  cv::Point arrow_end =
      arrow_start + cv::Point(static_cast<int>(velocity * cos(angle) * scale_),
                              static_cast<int>(velocity * sin(angle) * scale_));

  // 绘制速度箭头
  cv::arrowedLine(canvas_, arrow_start, arrow_end, cv::Scalar(0, 0, 255), 2, 8,
                  0, 0.3);  // 红色箭头表示速度

  // 显示车辆ID或其他信息
  cv::putText(canvas_, "ego", arrow_start, cv::FONT_HERSHEY_SIMPLEX, 0.5,
              cv::Scalar(0, 0, 255), 1);
}
void Visualizer::VisualizeSurroundingLaneNet(
    const double& stamp, const common::LaneNet& lane_net,
    const std::vector<int>& deleted_lane_ids) {}
void Visualizer::VisualizeSurroundingVehicles(
    const double& stamp, const common::VehicleSet& vehicle_set,
    const std::vector<int>& nearby_ids) {
  for (const auto& vehicle_pair : vehicle_set.vehicles) {
    const auto& vehicle = vehicle_pair.second;

    // 获取车辆的 OBB (Oriented BoundingBox)
    common::OrientedBoundingBox2D obb = vehicle.RetOrientedBoundingBox();

    // 获取车辆的四个顶点
    vec_E<Vec2f> vertices;
    common::SemanticsUtils::GetVehicleVertices(vehicle.param(), vehicle.state(),
                                               &vertices);

    // 将顶点转换为 cv::Point2i，并按照 scale 和 offset 进行缩放和平移
    std::vector<cv::Point> polygon_points;
    for (const auto& vertex : vertices) {
      polygon_points.push_back(
          cv::Point(static_cast<int>(vertex.x() * scale_ + offset_.x),
                    static_cast<int>(vertex.y() * scale_ + offset_.y)));
    }

    // 创建一个与画布相同大小的临时图层
    cv::Mat overlay;
    canvas_.copyTo(overlay);

    // 在临时图层上填充车辆轮廓 (半透明蓝色)
    cv::fillPoly(overlay, std::vector<std::vector<cv::Point>>{polygon_points},
                 cv::Scalar(255, 0, 0, 100));  // 100 表示透明度

    // 将临时图层与原始画布混合，生成半透明效果
    double alpha = 0.4;  // 控制透明度
    cv::addWeighted(overlay, alpha, canvas_, 1 - alpha, 0, canvas_);

    // 绘制车辆轮廓 (多边形)
    cv::polylines(canvas_, polygon_points, true, cv::Scalar(255, 0, 0),
                  2);  // 绿色线表示车辆

    // 绘制速度箭头
    const auto& state = vehicle.state();
    double velocity = state.velocity;  // 获取车辆的速度大小
    double angle = state.angle;        // 获取车辆的朝向

    // // 判断车辆是否在倒车 (根据速度和角度关系)
    // if (state.is_reverse) {  // 假设 `state.is_reverse` 标识车辆是否在倒车
    //   velocity = -velocity;  // 如果在倒车，反向调整速度的箭头方向
    // }

    // 计算速度箭头的起点和终点
    cv::Point arrow_start(static_cast<int>(obb.x * scale_ + offset_.x),
                          static_cast<int>(obb.y * scale_ + offset_.y));
    cv::Point arrow_end =
        arrow_start +
        cv::Point(static_cast<int>(velocity * cos(angle) * scale_),
                  static_cast<int>(velocity * sin(angle) * scale_));

    // 绘制速度箭头
    cv::arrowedLine(canvas_, arrow_start, arrow_end, cv::Scalar(255, 0, 0), 2,
                    8, 0, 0.3);  // 红色箭头表示速度

    // 显示车辆ID或其他信息
    cv::putText(canvas_, std::to_string(vehicle.id()), arrow_start,
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
  }
}

void Visualizer::VisualizeObstacleMap(
    const double& stamp, const common::GridMapND<uint8_t, 2>& obstacle_map) {
  // 获取地图的尺寸
  int width = obstacle_map.dims_size(1);   // x 方向
  int height = obstacle_map.dims_size(0);  // y 方向

  // 创建一个 OpenCV 图像来存储地图数据
  cv::Mat map_image(height, width, CV_8UC3,
                    cv::Scalar(255, 255, 255));  // 白色背景

  // 遍历地图中的每个栅格点
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      uint8_t value = obstacle_map.data(y * width + x);  // 获取栅格值

      // 根据栅格值设置像素颜色
      if (value == common::GridMapND<uint8_t, 2>::OCCUPIED) {
        map_image.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);  // 黑色表示障碍物
      } else if (value == common::GridMapND<uint8_t, 2>::SCANNED_OCCUPIED) {
        map_image.at<cv::Vec3b>(y, x) =
            cv::Vec3b(0, 0, 255);  // 红色表示已扫描的障碍物
      }
    }
  }

  // 放大地图
  cv::Mat enlarged_map_image;
  int scale_factor = 4;  // 放大倍数，可根据需要调整
  cv::resize(map_image, enlarged_map_image, cv::Size(), scale_factor,
             scale_factor, cv::INTER_NEAREST);

  // 显示放大的地图
  cv::imshow("Obstacle Map", enlarged_map_image);
  cv::waitKey(1);
}

void Visualizer::VisualizeStaticObstacle(
    const double& stamp, const common::ObstacleSet& obstacle_set) {
  // 可视化圆形障碍物
  for (const auto& obs : obstacle_set.obs_circle) {
    const auto& circle = obs.second.circle;

    // 将圆心坐标从全局坐标转换为画布坐标
    cv::Point2d center(circle.center.x * scale_ + offset_.x,
                       circle.center.y * scale_ + offset_.y);

    // 将半径转换为画布尺度
    int radius = static_cast<int>(circle.radius * scale_);

    // 绘制圆形障碍物
    cv::circle(canvas_, center, radius, cv::Scalar(0, 0, 255),
               2);  // 红色圆表示障碍物
  }

  // 可视化多边形障碍物
  for (const auto& obs : obstacle_set.obs_polygon) {
    std::vector<cv::Point> polygon_points;

    // 将多边形的每个顶点从全局坐标转换为画布坐标
    for (const auto& point : obs.second.polygon.points) {
      polygon_points.push_back(
          cv::Point(static_cast<int>(point.x * scale_ + offset_.x),
                    static_cast<int>(point.y * scale_ + offset_.y)));
    }

    // 绘制多边形障碍物
    cv::polylines(canvas_, polygon_points, true, cv::Scalar(0, 0, 255),
                  2);  // 红色线表示障碍物
  }
}

// 可视化 GridMap 的方法
void Visualizer::VisualizeGridMap(
    const std::set<std::array<decimal_t, 2>>& grid_map) {
  if (grid_map.empty()) {
    return;
  }
  // 计算地图尺寸 (假设根据坐标范围决定地图尺寸)
  decimal_t min_x = std::numeric_limits<decimal_t>::max();
  decimal_t max_x = std::numeric_limits<decimal_t>::lowest();
  decimal_t min_y = std::numeric_limits<decimal_t>::max();
  decimal_t max_y = std::numeric_limits<decimal_t>::lowest();

  for (const auto& grid_point : grid_map) {
    min_x = std::min(min_x, grid_point[0]);
    max_x = std::max(max_x, grid_point[0]);
    min_y = std::min(min_y, grid_point[1]);
    max_y = std::max(max_y, grid_point[1]);
  }

  int width = static_cast<int>(max_x - min_x + 1);
  int height = static_cast<int>(max_y - min_y + 1);

  // 创建一个 OpenCV 图像来存储地图数据
  cv::Mat grid_image(height, width, CV_8UC3,
                     cv::Scalar(255, 255, 255));  // 白色背景

  // 遍历自定义的栅格点并绘制
  for (const auto& grid_point : grid_map) {
    int x = static_cast<int>(grid_point[0] - min_x);
    int y = static_cast<int>(grid_point[1] - min_y);

    // 绘制绿色小方块表示栅格
    grid_image.at<cv::Vec3b>(y, x) =
        cv::Vec3b(0, 255, 0);  // 绿色像素表示自定义栅格
  }

  // 放大地图
  cv::Mat enlarged_grid_image;
  int scale_factor = 10;  // 放大倍数，可根据需要调整
  cv::resize(grid_image, enlarged_grid_image, cv::Size(), scale_factor,
             scale_factor, cv::INTER_NEAREST);

  // 显示放大的地图
  cv::imshow("Grid Map", enlarged_grid_image);
  cv::waitKey(1);
}

}  // namespace semantic_map_manager
