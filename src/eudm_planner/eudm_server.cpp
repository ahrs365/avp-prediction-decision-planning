#include <thread>

#include "eudm_server.h"
planning::EudmServer::EudmServer() {}

planning::EudmServer::~EudmServer() {}

void planning::EudmServer::run(double cycle_time_ms) {
  while (true) {
    auto start_time = std::chrono::steady_clock::now();

    SemanticMapManager smm;

    // 等待SMM地图
    {
      std::unique_lock<std::mutex> lock(*mutex_);
      cv_->wait(lock, [&] { return !smmQueue_->empty(); });
      smm = smmQueue_->front();
      smmQueue_->pop();
    }

    // // 生成轨迹
    // Trajectory traj = generateTrajectory(smm);

    // // 发送轨迹
    // sendTrajectory(traj);

    // 控制规划线程周期
    std::this_thread::sleep_until(
        start_time +
        std::chrono::milliseconds(static_cast<int>(cycle_time_ms)));
  }
}

void planning::EudmServer::setQueue(std::queue<SemanticMapManager>& smmQueue,
                                    std::mutex& mutex,
                                    std::condition_variable& cv) {
  smmQueue_ = &smmQueue;
  mutex_ = &mutex;
  cv_ = &cv;
}
