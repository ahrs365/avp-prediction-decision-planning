#ifndef ENVIRONMENT_HPP
#define ENVIRONMENT_HPP

#include <string>
#include <unordered_map>
#include <vector>

#include "park_data_reader/loader.h"

namespace park {
class Environment {
 public:
  Environment(const std::unordered_map<std::string, Obstacle>& obstacles,
              const std::unordered_map<std::string, Frame>& frames,
              const std::unordered_map<std::string, Instance>& instances,
              const std::unordered_map<std::string, Agent>& agents,
              const ParkingMap& parkingMap);

  void loadFrame(const Frame& frame);
  const std::vector<Obstacle>& getCurrentObstacles() const;
  const std::vector<Agent>& getCurrentAgents() const;
  const Instance* getInstance(const std::string& instance_token) const;

  void printParkingMapInfo() const;  // 添加打印停车位和航路点信息的方法

 private:
  std::unordered_map<std::string, Obstacle> allObstacles;
  std::unordered_map<std::string, Frame> allFrames;
  std::unordered_map<std::string, Instance> allInstances;
  std::unordered_map<std::string, Agent> allAgents;
  ParkingMap parkingMap;

  std::vector<Obstacle> currentObstacles;
  std::vector<Agent> currentAgents;
};

}  // namespace park

#endif  // ENVIRONMENT_HPP
