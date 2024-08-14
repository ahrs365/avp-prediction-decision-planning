#pragma once

#include "semantic_map_manager/map_adapter.h"
using namespace semantic_map_manager;

namespace planning {
class EudmServer {
 public:
  EudmServer();
  ~EudmServer();

  void run(SemanticMapManager* p_smm);

 private:
  SemanticMapManager* p_smm_;

  std::vector<double> generateTrajectory(const SemanticMapManager& smm);
  void sendTrajectory(const std::vector<double>& traj);
};

}  // namespace planning