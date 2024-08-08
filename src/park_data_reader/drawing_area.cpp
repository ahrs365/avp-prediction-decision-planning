#include <FL/fl_draw.H>

#include "drawing_area.h"
#include "park_simulation.h"

namespace park {

DrawingArea::DrawingArea(int X, int Y, int W, int H, const char* L)
    : Fl_Widget(X, Y, W, H, L), env(nullptr), staticElementsDrawn(false) {}

void DrawingArea::setEnvironment(Environment* env) { this->env = env; }

void DrawingArea::setParkingMap(ParkingMap map) { this->parkingMap = map; }

void DrawingArea::setCurrentFrame(const Frame& frame) {
  this->frame = frame;
  redraw();
}

void DrawingArea::draw() {
  // 清除背景
  fl_color(FL_WHITE);
  fl_rectf(x(), y(), w(), h());

  if (!staticElementsDrawn) {
    drawStaticElements();
    staticElementsDrawn = true;
  }
  drawFrame();
}

void DrawingArea::drawStaticElements() {
  // 获取绘图区域的大小
  int draw_width = w();
  int draw_height = h();

  // 获取地图尺寸
  auto map_size = parkingMap.getMapSize();

  // 设置比例因子
  double scale_x = draw_width / static_cast<double>(map_size.first);
  double scale_y = draw_height / static_cast<double>(map_size.second);
  double scale = std::min(scale_x, scale_y);

  // 绘制停车位
  auto parkingSpots = parkingMap.getParkingSpots();
  for (const auto& area : parkingSpots) {
    for (const auto& spot : area.second) {
      for (size_t i = 0; i < spot.corners.size(); ++i) {
        size_t j = (i + 1) % spot.corners.size();
        fl_color(FL_GRAY);
        fl_line(x() + spot.corners[i].first * scale,
                y() + spot.corners[i].second * scale,
                x() + spot.corners[j].first * scale,
                y() + spot.corners[j].second * scale);
      }
    }
  }

  // 绘制静态障碍物
  const auto& staticObstacles = env->getCurrentStaticObstacles();
  for (const auto& pair : staticObstacles) {
    const auto& obstacle = pair.second;
    double theta = obstacle.heading;
    double half_length = obstacle.size[0] / 2;
    double half_width = obstacle.size[1] / 2;
    std::vector<std::pair<double, double>> corners = {
        {obstacle.coords[0] + half_length * std::cos(theta) -
             half_width * std::sin(theta),
         obstacle.coords[1] + half_length * std::sin(theta) +
             half_width * std::cos(theta)},
        {obstacle.coords[0] - half_length * std::cos(theta) -
             half_width * std::sin(theta),
         obstacle.coords[1] - half_length * std::sin(theta) +
             half_width * std::cos(theta)},
        {obstacle.coords[0] - half_length * std::cos(theta) +
             half_width * std::sin(theta),
         obstacle.coords[1] - half_length * std::sin(theta) -
             half_width * std::cos(theta)},
        {obstacle.coords[0] + half_length * std::cos(theta) +
             half_width * std::sin(theta),
         obstacle.coords[1] + half_length * std::sin(theta) -
             half_width * std::cos(theta)}};

    fl_color(FL_RED);
    for (size_t i = 0; i < corners.size(); ++i) {
      size_t j = (i + 1) % corners.size();
      fl_line(x() + corners[i].first * scale, y() + corners[i].second * scale,
              x() + corners[j].first * scale, y() + corners[j].second * scale);
    }
  }

  // 绘制航路点
  const auto& routePoints = parkingMap.getAllRoutePoints();
  for (const auto& point : routePoints) {
    fl_color(FL_BLACK);
    fl_point(x() + point.first * scale, y() + point.second * scale);
  }
}

void DrawingArea::drawFrame() {
  // 获取绘图区域的大小
  int draw_width = w();
  int draw_height = h();

  // 获取地图尺寸
  auto map_size = parkingMap.getMapSize();

  // 设置比例因子
  double scale_x = draw_width / static_cast<double>(map_size.first);
  double scale_y = draw_height / static_cast<double>(map_size.second);
  double scale = std::min(scale_x, scale_y);

  // 清除动态元素绘制区域
  fl_color(FL_WHITE);
  fl_rectf(x(), y(), w(), h());

  // 绘制静态元素以防被覆盖
  drawStaticElements();

  // 绘制动态障碍物
  const auto& currentDynamicObstacles = env->getCurrentDynamicObstacles();
  for (const auto& pair : currentDynamicObstacles) {
    const auto& obstacle = pair.second;
    double theta = obstacle.heading;
    double half_length = obstacle.size[0] / 2;
    double half_width = obstacle.size[1] / 2;
    std::vector<std::pair<double, double>> corners = {
        {obstacle.coords[0] + half_length * std::cos(theta) -
             half_width * std::sin(theta),
         obstacle.coords[1] + half_length * std::sin(theta) +
             half_width * std::cos(theta)},
        {obstacle.coords[0] - half_length * std::cos(theta) -
             half_width * std::sin(theta),
         obstacle.coords[1] - half_length * std::sin(theta) +
             half_width * std::cos(theta)},
        {obstacle.coords[0] - half_length * std::cos(theta) +
             half_width * std::sin(theta),
         obstacle.coords[1] - half_length * std::sin(theta) -
             half_width * std::cos(theta)},
        {obstacle.coords[0] + half_length * std::cos(theta) +
             half_width * std::sin(theta),
         obstacle.coords[1] + half_length * std::sin(theta) -
             half_width * std::cos(theta)}};

    fl_color(FL_BLUE);
    for (size_t i = 0; i < corners.size(); ++i) {
      size_t j = (i + 1) % corners.size();
      fl_line(x() + corners[i].first * scale, y() + corners[i].second * scale,
              x() + corners[j].first * scale, y() + corners[j].second * scale);
    }

    // 绘制速度向量
    double arrow_length = 0.5 * obstacle.speed;
    double arrow_x = obstacle.coords[0] + arrow_length * std::cos(theta);
    double arrow_y = obstacle.coords[1] + arrow_length * std::sin(theta);
    fl_line(x() + obstacle.coords[0] * scale, y() + obstacle.coords[1] * scale,
            x() + arrow_x * scale, y() + arrow_y * scale);

    // 绘制箭头头部
    double arrow_head_length = 0.1 * arrow_length;
    double arrow_head_angle = M_PI / 6;  // 30 degrees
    double head_x1 =
        arrow_x - arrow_head_length * std::cos(theta - arrow_head_angle);
    double head_y1 =
        arrow_y - arrow_head_length * std::sin(theta - arrow_head_angle);
    double head_x2 =
        arrow_x - arrow_head_length * std::cos(theta + arrow_head_angle);
    double head_y2 =
        arrow_y - arrow_head_length * std::sin(theta + arrow_head_angle);
    fl_line(x() + arrow_x * scale, y() + arrow_y * scale, x() + head_x1 * scale,
            y() + head_y1 * scale);
    fl_line(x() + arrow_x * scale, y() + arrow_y * scale, x() + head_x2 * scale,
            y() + head_y2 * scale);
  }
}

}  // namespace park
