#include <iostream>
#include <limits>

#include "park_data_reader/park_simulation.h"

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
  obstacles = Obstacle::loadFromFile(obstaclesFile, frames);
  parkingMap = ParkingMap::loadFromFile(mapFile);
  env = new Environment(obstacles, frames, instances, agents, parkingMap);

  // 打印环境中的停车位和航路点信息
  env->printParkingMapInfo();
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
    const auto& currentObstacles = env->getCurrentObstacles();
    const auto& currentAgents = env->getCurrentAgents();

    // 输出当前帧的信息
    std::cout << "\n=============================================\n";
    std::cout << "Current Frame: " << frame.frame_token << std::endl;
    std::cout << "  Timestamp: " << frame.timestamp << std::endl;
    std::cout << "  Next Frame: " << frame.next << std::endl;

    // 输出当前帧的障碍物信息
    std::cout << "  Obstacles in this frame:" << std::endl;
    if (currentObstacles.empty()) {
      std::cout << "    No obstacles in this frame." << std::endl;
    } else {
      for (const auto& obstacle : currentObstacles) {
        std::cout << "    Obstacle: " << obstacle.obstacle_token
                  << ", Type: " << obstacle.type << std::endl;
      }
    }

    // 输出当前帧的代理信息
    std::cout << "  Agents in this frame:" << std::endl;
    if (currentAgents.empty()) {
      std::cout << "    No agents in this frame." << std::endl;
    } else {
      for (const auto& agent : currentAgents) {
        const Instance* instance = env->getInstance(agent.first_instance);
        if (instance) {
          std::cout << "    Agent: " << agent.agent_token
                    << "\n      Type: " << agent.type
                    << "\n      Speed: " << instance->speed
                    << "\n      Position: (" << instance->coords[0] << ", "
                    << instance->coords[1] << ")"
                    << "\n      Size: (" << agent.size[0] << ", "
                    << agent.size[1] << ")\n";
        } else {
          std::cout << "    Agent: " << agent.agent_token
                    << "\n      Type: " << agent.type << "\n      Speed: N/A"
                    << "\n      Position: (N/A, N/A)"
                    << "\n      Size: (" << agent.size[0] << ", "
                    << agent.size[1] << ")\n";
        }
      }
    }

    // 移动到下一帧
    currentFrameToken = frame.next;
  }

  // 清理环境对象
  delete env;
}

}  // namespace park
