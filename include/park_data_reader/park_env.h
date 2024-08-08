#ifndef ENVIRONMENT_HPP
#define ENVIRONMENT_HPP

#include <string>
#include <unordered_map>
#include <vector>

#include "park_data_reader/loader.h"
#include "park_data_reader/parking_map.h"
namespace park {

struct StaticObstacle {
 public:
  std::string obstacle_token;
  std::string type;
  std::vector<double> size;
  std::vector<double> coords;
  double heading;
};

struct DynamicObstacle {
  std::string agent_token;
  std::string type;
  std::vector<double> size;
  std::vector<double> coords;
  double heading;
  double speed;
  std::vector<double> acceleration;
};

class Environment {
 public:
  Environment(const std::unordered_map<std::string, Obstacle>& obstacles,
              const std::unordered_map<std::string, Frame>& frames,
              const std::unordered_map<std::string, Instance>& instances,
              const std::unordered_map<std::string, Agent>& agents,
              const ParkingMap& parkingMap);

  void loadFrame(const Frame& frame);
  const std::unordered_map<std::string, StaticObstacle>&
  getCurrentStaticObstacles() const;
  const std::unordered_map<std::string, DynamicObstacle>&
  getCurrentDynamicObstacles() const;
  const Instance* getInstance(const std::string& instance_token) const;

 private:
  std::unordered_map<std::string, Frame> allFrames;
  std::unordered_map<std::string, Instance> allInstances;
  std::unordered_map<std::string, Agent> allAgents;

  ParkingMap parkingMap;

  std::unordered_map<std::string, StaticObstacle> allStaticObstacles;
  std::unordered_map<std::string, DynamicObstacle> allDynamicObstacles;
};

}  // namespace park

#endif  // ENVIRONMENT_HPP
