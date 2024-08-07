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
              const std::unordered_map<std::string, Agent>& agents);

  void loadFrame(const Frame& frame);
  const std::vector<Obstacle>& getCurrentObstacles() const;
  const std::vector<Agent>& getCurrentAgents() const;
  const Instance* getInstance(const std::string& instance_token) const;

 private:
  std::unordered_map<std::string, Obstacle> allObstacles;
  std::unordered_map<std::string, Frame> allFrames;
  std::unordered_map<std::string, Instance> allInstances;
  std::unordered_map<std::string, Agent> allAgents;

  std::vector<Obstacle> currentObstacles;
  std::vector<Agent> currentAgents;
};

}  // namespace park
#endif  // ENVIRONMENT_HPP
