#include <iostream>

#include "park_data_reader/park_simulation.h"

int main() {
  park::ParkSimulation simulation(
      "./data/DJI_0002_scene.json", "./data/DJI_0002_agents.json",
      "./data/DJI_0002_frames.json", "./data/DJI_0002_instances.json",
      "./data/DJI_0002_obstacles.json", 0, 10  // startFrameIndex, endFrameIndex
  );

  simulation.run();

  return 0;

  //   // 加载场景数据
  //   auto scene = park::Scene::loadFromFile("./data/DJI_0002_scene.json");

  //   // 指定起始和终止的 frame 索引
  //   int start_index = 0;  // 替换为实际的起始索引
  //   int end_index = 10;   // 替换为实际的终止索引

  //   // 加载所有数据
  //   auto agents = park::Agent::loadFromFile("./data/DJI_0002_agents.json");
  //   auto frames = park::Frame::loadFromFile("./data/DJI_0002_frames.json",
  //                                           start_index, end_index);
  //   auto instances =
  //       park::Instance::loadFromFile("./data/DJI_0002_instances.json",
  //       frames);
  //   auto obstacles =
  //       park::Obstacle::loadFromFile("./data/DJI_0002_obstacles.json",
  //       frames);

  //   // 创建环境对象
  //   park::Environment env(obstacles, frames, instances, agents);

  //   // 找到时间戳最小的帧作为起始帧
  //   std::string currentFrameToken;
  //   double minTimestamp = std::numeric_limits<double>::max();

  //   for (const auto& frame_pair : frames) {
  //     const auto& frame = frame_pair.second;
  //     if (frame.timestamp < minTimestamp) {
  //       minTimestamp = frame.timestamp;
  //       currentFrameToken = frame.frame_token;
  //     }
  //   }

  //   // 按照 next 的顺序回放帧
  //   while (!currentFrameToken.empty()) {
  //     auto it = frames.find(currentFrameToken);
  //     if (it == frames.end()) {
  //       std::cerr << "Error: Frame token " << currentFrameToken
  //                 << " not found in frames map." << std::endl;
  //       break;
  //     }
  //     const auto& frame = it->second;
  //     env.loadFrame(frame);
  //     const auto& currentObstacles = env.getCurrentObstacles();
  //     const auto& currentAgents = env.getCurrentAgents();

  //     // 输出当前帧的信息
  //     std::cout << "\n=============================================\n";
  //     std::cout << "Current Frame: " << frame.frame_token << std::endl;
  //     std::cout << "  Timestamp: " << frame.timestamp << std::endl;
  //     std::cout << "  Next Frame: " << frame.next << std::endl;

  //     // 输出当前帧的障碍物信息
  //     std::cout << "  Obstacles in this frame:" << std::endl;
  //     if (currentObstacles.empty()) {
  //       std::cout << "    No obstacles in this frame." << std::endl;
  //     } else {
  //       for (const auto& obstacle : currentObstacles) {
  //         std::cout << "    Obstacle: " << obstacle.obstacle_token
  //                   << ", Type: " << obstacle.type << std::endl;
  //       }
  //     }

  //     // 输出当前帧的代理信息
  //     std::cout << "  Agents in this frame:" << std::endl;
  //     if (currentAgents.empty()) {
  //       std::cout << "    No agents in this frame." << std::endl;
  //     } else {
  //       for (const auto& agent : currentAgents) {
  //         const park::Instance* instance =
  //         env.getInstance(agent.first_instance); if (instance) {
  //           std::cout << "    Agent: " << agent.agent_token
  //                     << "\n      Type: " << agent.type
  //                     << "\n      Speed: " << instance->speed
  //                     << "\n      Position: (" << instance->coords[0] << ", "
  //                     << instance->coords[1] << ")"
  //                     << "\n      Size: (" << agent.size[0] << ", "
  //                     << agent.size[1] << ")\n";
  //         } else {
  //           std::cout << "    Agent: " << agent.agent_token
  //                     << "\n      Type: " << agent.type << "\n      Speed:
  //                     N/A"
  //                     << "\n      Position: (N/A, N/A)"
  //                     << "\n      Size: (" << agent.size[0] << ", "
  //                     << agent.size[1] << ")\n";
  //         }
  //       }
  //     }

  //     // 移动到下一帧
  //     currentFrameToken = frame.next;
  //   }

  //   return 0;
}