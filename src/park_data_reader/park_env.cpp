#include <iostream>

#include "park_data_reader/park_env.h"

namespace park {
Environment::Environment(
    const std::unordered_map<std::string, Obstacle>& obstacles,
    const std::unordered_map<std::string, Frame>& frames,
    const std::unordered_map<std::string, Instance>& instances,
    const std::unordered_map<std::string, Agent>& agents,
    const ParkingMap& parkingMap)
    : allFrames(frames),
      allInstances(instances),
      allAgents(agents),
      parkingMap(parkingMap) {
  // 在构造函数中加载所有障碍物，因为它们是静态的
  allStaticObstacles.clear();
  for (const auto& obstacle : obstacles) {
    StaticObstacle static_obstacle;
    static_obstacle.obstacle_token = obstacle.first;
    static_obstacle.coords = obstacle.second.coords;
    static_obstacle.heading = obstacle.second.heading;
    static_obstacle.size = obstacle.second.size;
    static_obstacle.type = obstacle.second.type;
    allStaticObstacles.insert(std::make_pair(obstacle.first, static_obstacle));
  }
}

void Environment::loadFrame(const Frame& frame) {
  allDynamicObstacles.clear();
  setTimeStamp(frame.timestamp);
  for (const auto& instance_token : frame.instances) {
    if (allInstances.find(instance_token) != allInstances.end()) {
      const auto& instance = allInstances[instance_token];
      if (allAgents.find(instance.agent_token) != allAgents.end()) {
        const auto& agent = allAgents[instance.agent_token];
        DynamicObstacle dynamic_obstacle;
        dynamic_obstacle.agent_token = agent.agent_token;
        dynamic_obstacle.coords = instance.coords;
        dynamic_obstacle.heading = instance.heading;
        dynamic_obstacle.size = agent.size;
        dynamic_obstacle.speed = instance.speed;
        dynamic_obstacle.type = agent.type;
        dynamic_obstacle.acceleration = instance.acceleration;
        allDynamicObstacles.insert(
            std::make_pair(dynamic_obstacle.agent_token, dynamic_obstacle));
      }
    }
  }

  // Debug output to check if obstacles and agents are being loaded
  // std::cout << "Loaded frame " << frame.frame_token << " with "
  //           << allStaticObstacles.size() << " static obstacles and "
  //           << allDynamicObstacles.size() << " dynamic obstacles." <<
  //           std::endl;
}

const std::unordered_map<std::string, StaticObstacle>&
Environment::getCurrentStaticObstacles() const {
  return allStaticObstacles;
}

const std::unordered_map<std::string, DynamicObstacle>&
Environment::getCurrentDynamicObstacles() const {
  return allDynamicObstacles;
}

const Instance* Environment::getInstance(
    const std::string& instance_token) const {
  auto it = allInstances.find(instance_token);
  if (it != allInstances.end()) {
    return &(it->second);
  }
  return nullptr;
}

const double Environment::getTimeStamp() const { return time_stamp_; }
void Environment::setTimeStamp(const double& time_stamp) {
  time_stamp_ = time_stamp;
}
const ParkingMap park::Environment::getParkingMap() const { return parkingMap; }

}  // namespace park
