#pragma once
#include <assert.h>

#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>
#include <vector>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "park_data_reader/park_simulation.h"
#include "semantic_map_manager/data_renderer.h"
#include "semantic_map_manager/semantic_map_manager.h"

using namespace park;

namespace semantic_map_manager {
class MapAdapter {
 public:
  MapAdapter(SemanticMapManager* ptr_smm);
  ~MapAdapter();
  void run(double cycle_time_ms);

  void setQueues(std::queue<Environment*>& envQueue,
                 std::queue<ParkingMap*>& parkingMapQueue,
                 std::queue<SemanticMapManager*>& smmQueue, std::mutex& mutex,
                 std::condition_variable& cv);

 private:
  std::queue<Environment*>* envQueue_;
  std::queue<ParkingMap*>* parkingMapQueue_;
  std::queue<SemanticMapManager*>* smmQueue_;
  std::mutex* mutex_;
  std::condition_variable* cv_;

  common::Vehicle ego_vehicle_;
  common::VehicleSet vehicle_set_;
  common::LaneNet lane_net_;
  common::ObstacleSet obstacle_set_;
  DataRenderer* p_data_renderer_;
  double time_stamp_;
  SemanticMapManager* p_smm_;
  bool get_arena_info_static_ = false;

  void GetSimulationDataFromStatic(Environment* env);
  void GetSimulationDataFromDynamic(Environment* env);
  void GetPolygonFromSimulationData(const StaticObstacle& obs,
                                    common::PolygonObstacle* poly);
  void GetVehicleFromSimulationData(const DynamicObstacle& obs,
                                    common::Vehicle* vehicle);
  void updateMap(Environment* env, ParkingMap* parkingMap);
};
}  // namespace semantic_map_manager