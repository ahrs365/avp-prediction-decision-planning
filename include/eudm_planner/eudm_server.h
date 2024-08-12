#pragma once
#include <condition_variable>
#include <mutex>
#include <queue>

#include "semantic_map_manager/map_adapter.h"
using namespace semantic_map_manager;

namespace planning {
class EudmServer {
 public:
  EudmServer();
  ~EudmServer();

  void run(double cycle_time_ms);

  void setQueue(std::queue<SemanticMapManager*>& smmQueue, std::mutex& mutex,
                std::condition_variable& cv);

 private:
  std::queue<SemanticMapManager*>* smmQueue_;
  std::mutex* mutex_;
  std::condition_variable* cv_;

  std::vector<double> generateTrajectory(const SemanticMapManager& smm);
  void sendTrajectory(const std::vector<double>& traj);
};

}  // namespace planning