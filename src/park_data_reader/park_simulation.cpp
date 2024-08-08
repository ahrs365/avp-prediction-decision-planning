#include <iostream>
#include <limits>

#include "matplotlibcpp.h"
#include "park_data_reader/park_simulation.h"
namespace plt = matplotlibcpp;
namespace park {

ParkSimulation::ParkSimulation(const std::string& sceneFile,
                               const std::string& agentsFile,
                               const std::string& framesFile,
                               const std::string& instancesFile,
                               const std::string& obstaclesFile,
                               const std::string& mapFile, int startFrameIndex,
                               int endFrameIndex)
    : sceneFile(sceneFile),
      agentsFile(agentsFile),
      framesFile(framesFile),
      instancesFile(instancesFile),
      obstaclesFile(obstaclesFile),
      mapFile(mapFile),
      startFrameIndex(startFrameIndex),
      endFrameIndex(endFrameIndex),
      env(nullptr) {}

void ParkSimulation::loadData() {
  scene = Scene::loadFromFile(sceneFile);
  agents = Agent::loadFromFile(agentsFile);
  frames = Frame::loadFromFile(framesFile, startFrameIndex, endFrameIndex);
  instances = Instance::loadFromFile(instancesFile, frames);
  obstacles = Obstacle::loadFromFile(obstaclesFile);
  parkingMap = ParkingMap::loadFromFile(mapFile);
  env = new Environment(obstacles, frames, instances, agents, parkingMap);
}

void ParkSimulation::findStartFrame() {
  double minTimestamp = std::numeric_limits<double>::max();

  for (const auto& frame_pair : frames) {
    const auto& frame = frame_pair.second;
    if (frame.timestamp < minTimestamp) {
      minTimestamp = frame.timestamp;
      currentFrameToken = frame.frame_token;
    }
  }
}

void ParkSimulation::run() {
  loadData();
  findStartFrame();

  while (!currentFrameToken.empty()) {
    auto it = frames.find(currentFrameToken);
    if (it == frames.end()) {
      std::cerr << "Error: Frame token " << currentFrameToken
                << " not found in frames map." << std::endl;
      break;
    }
    const auto& frame = it->second;
    env->loadFrame(frame);

    // 输出当前帧的信息
    std::cout << "\n=============================================\n";
    std::cout << "Current Frame: " << frame.frame_token << std::endl;
    std::cout << "  Timestamp: " << frame.timestamp << std::endl;
    std::cout << "  Next Frame: " << frame.next << std::endl;

    // 绘制当前帧
    drawFrame(frame);
    // 移动到下一帧
    currentFrameToken = frame.next;
  }

  // 清理环境对象
  delete env;
}

void ParkSimulation::drawStaticElements() {
  // 绘制停车位
  auto parkingSpots = parkingMap.getParkingSpots();
  for (const auto& area : parkingSpots) {
    for (const auto& spot : area.second) {
      for (size_t i = 0; i < spot.corners.size(); ++i) {
        size_t j = (i + 1) % spot.corners.size();
        plt::plot({spot.corners[i].first, spot.corners[j].first},
                  {spot.corners[i].second, spot.corners[j].second}, "gray");
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

    for (size_t i = 0; i < corners.size(); ++i) {
      size_t j = (i + 1) % corners.size();
      plt::plot({corners[i].first, corners[j].first},
                {corners[i].second, corners[j].second}, "r-");
    }
  }

  // 绘制航路点
  const auto& routePoints = parkingMap.getAllRoutePoints();
  for (const auto& point : routePoints) {
    plt::scatter(std::vector<double>{point.first},
                 std::vector<double>{point.second}, 10.0);
  }

  // 设置图形范围
  plt::xlim(0, parkingMap.getMapSize().first);
  plt::ylim(0, parkingMap.getMapSize().second);
  plt::pause(0.01);  // 暂停以更新图形
}

void ParkSimulation::drawFrame(const Frame& frame) {
  if (!staticElementsDrawn) {
    drawStaticElements();
    staticElementsDrawn = true;
  }
  plt::clf();  // 清除当前图形

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

    std::string color;
    if (obstacle.type == "Car") {
      color = "b-";
    } else if (obstacle.type == "Pedestrian") {
      color = "g-";
    } else {
      color = "y-";
    }

    for (size_t i = 0; i < corners.size(); ++i) {
      size_t j = (i + 1) % corners.size();
      plt::plot({corners[i].first, corners[j].first},
                {corners[i].second, corners[j].second}, color);
    }

    // 绘制速度向量
    double arrow_length = 0.5 * obstacle.speed;
    double arrow_x = obstacle.coords[0] + arrow_length * std::cos(theta);
    double arrow_y = obstacle.coords[1] + arrow_length * std::sin(theta);
    plt::plot({obstacle.coords[0], arrow_x}, {obstacle.coords[1], arrow_y},
              color);

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
    plt::plot({arrow_x, head_x1}, {arrow_y, head_y1}, color);
    plt::plot({arrow_x, head_x2}, {arrow_y, head_y2}, color);
  }

  plt::xlim(0, parkingMap.getMapSize().first);
  plt::ylim(0, parkingMap.getMapSize().second);
  plt::pause(0.01);  // 暂停以更新图形
}

}  // namespace park
