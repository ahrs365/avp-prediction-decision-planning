#include <FL/Fl.H>

#include <chrono>  // 包含 <chrono> 头文件
#include <iostream>
#include <limits>
#include <thread>  // 包含 <thread> 头文件

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

void ParkSimulation::run(double cycle_time_ms) {
  loadData();
  findStartFrame();

  while (!currentFrameToken.empty()) {
    auto start_time = std::chrono::steady_clock::now();
    auto it = frames.find(currentFrameToken);
    if (it == frames.end()) {
      break;
    }
    const auto& frame = it->second;
    env->loadFrame(frame);
    const size_t max_queue_size = 10;  // 设置队列的最大容量
    // 输出当前帧的信息
    // std::cout << "\n=============================================\n";
    // std::cout << "Current Frame: " << frame.frame_token << std::endl;
    // std::cout << "  Timestamp: " << frame.timestamp << std::endl;
    // std::cout << "  Next Frame: " << frame.next << std::endl;

    // // 更新绘图
    // drawingArea->setEnvironment(env);
    // drawingArea->setParkingMap(parkingMap);
    // drawingArea->setCurrentFrame(frame);
    // drawingArea->redraw();
    // Fl::check();  // 使用 FLTK 的 check 方法

    // 如果队列已满，删除最早的元素
    {
      std::lock_guard<std::mutex> lock(*mutex_);
      if (envQueue_->size() >= max_queue_size) {
        envQueue_->pop();  // 删除最早的元素
      }
      if (parkingMapQueue_->size() >= max_queue_size) {
        parkingMapQueue_->pop();
      }
      envQueue_->push(env);
      parkingMapQueue_->push(&parkingMap);
    }
    cv_->notify_all();

    // 控制回放线程周期
    std::this_thread::sleep_until(
        start_time +
        std::chrono::milliseconds(static_cast<int>(cycle_time_ms)));

    // 移动到下一帧
    currentFrameToken = frame.next;
    auto end_time = std::chrono::steady_clock::now();
    std::cout << "ParkSimulation::run() cycle time: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(
                     end_time - start_time)
                     .count()
              << " ms" << std::endl;
  }

  std::cout << "Simulation finished." << std::endl;
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

void ParkSimulation::setQueues(std::queue<Environment*>& envQueue,
                               std::queue<ParkingMap*>& parkingMapQueue,
                               std::mutex& mutex, std::condition_variable& cv) {
  envQueue_ = &envQueue;
  parkingMapQueue_ = &parkingMapQueue;
  mutex_ = &mutex;
  cv_ = &cv;
}

}  // namespace park
