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

void Environment::printParkingMapInfo() const {
  std::cout << "Parking Map Size: " << parkingMap.map_size.first << " x "
            << parkingMap.map_size.second << std::endl;

  std::cout << "Parking Areas:" << std::endl;
  for (const auto& [area_name, area] : parkingMap.parking_areas) {
    std::cout << "  Area " << area_name << " bounds:" << std::endl;
    for (const auto& bound : area.bounds) {
      std::cout << "    [" << bound[0] << ", " << bound[1] << "]" << std::endl;
    }
    for (const auto& a : area.areas) {
      std::cout << "    Shape: [" << a.second.first << ", " << a.second.second
                << "]" << std::endl;
      if (!a.first.empty()) {
        std::cout << "    Coords: ";
        for (const auto& coord : a.first) {
          std::cout << coord << " ";
        }
        std::cout << std::endl;
      }
    }
  }

  std::cout << "Waypoints:" << std::endl;
  for (const auto& [waypoint_name, waypoint] : parkingMap.waypoints) {
    std::cout << "  Waypoint " << waypoint_name << " from ["
              << waypoint.start[0] << ", " << waypoint.start[1] << "] to ["
              << waypoint.end[0] << ", " << waypoint.end[1] << "] with "
              << waypoint.nums << " points." << std::endl;
  }
}

}  // namespace park
