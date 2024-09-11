#include <iostream>
#include <thread>

#include "eudm_planner/eudm_server.h"
planning::EudmServer::EudmServer() {}

planning::EudmServer::~EudmServer() {}

void planning::EudmServer::run(SemanticMapManager* p_smm) {
  p_smm_ = p_smm;
  auto start_time = std::chrono::steady_clock::now();
  // // 生成轨迹
  // Trajectory traj = generateTrajectory(smm);

  // // 发送轨迹
  // sendTrajectory(traj);

  auto end_time = std::chrono::steady_clock::now();
  std::cout << "EudmServer::run() cycle time: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end_time -
                                                                     start_time)
                   .count()
            << "ms" << std::endl;
}
