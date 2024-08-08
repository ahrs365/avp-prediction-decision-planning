#include <iostream>

#include "park_data_reader/park_env.h"

namespace park {
Environment::Environment(
    const std::unordered_map<std::string, Obstacle>& obstacles,
    const std::unordered_map<std::string, Frame>& frames,
    const std::unordered_map<std::string, Instance>& instances,
    const std::unordered_map<std::string, Agent>& agents,
    const ParkingMap& parkingMap)
    : allObstacles(obstacles),
      allFrames(frames),
      allInstances(instances),
      allAgents(agents),
      parkingMap(parkingMap) {}

void Environment::loadFrame(const Frame& frame) {
  currentObstacles.clear();
  currentAgents.clear();
  for (const auto& instance_token : frame.instances) {
    if (allInstances.find(instance_token) != allInstances.end()) {
      const auto& instance = allInstances[instance_token];
      if (allObstacles.find(instance_token) != allObstacles.end()) {
        const auto& obstacle = allObstacles[instance_token];
        currentObstacles.push_back(obstacle);
      }
      if (allAgents.find(instance.agent_token) != allAgents.end()) {
        const auto& agent = allAgents[instance.agent_token];
        currentAgents.push_back(agent);
      }
    }
  }
  // Debug output to check if obstacles and agents are being loaded
  std::cout << "Loaded frame " << frame.frame_token << " with "
            << currentObstacles.size() << " obstacles and "
            << currentAgents.size() << " agents." << std::endl;
}

const std::vector<Obstacle>& Environment::getCurrentObstacles() const {
  return currentObstacles;
}

const std::vector<Agent>& Environment::getCurrentAgents() const {
  return currentAgents;
}

const Instance* Environment::getInstance(
    const std::string& instance_token) const {
  auto it = allInstances.find(instance_token);
  if (it != allInstances.end()) {
    return &(it->second);
  }
  return nullptr;
}

}  // namespace park
