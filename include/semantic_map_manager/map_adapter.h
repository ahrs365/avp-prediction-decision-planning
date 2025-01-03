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
#include "common/lane/graph.h"
#include "park_data_reader/park_simulation.h"
#include "semantic_map_manager/data_renderer.h"
#include "semantic_map_manager/semantic_map_manager.h"
using namespace park;

namespace semantic_map_manager {
class MapAdapter {
 public:
  MapAdapter(SemanticMapManager* ptr_smm);
  ~MapAdapter();
  void run(Environment* env);

 private:
  common::Vehicle ego_vehicle_;
  common::VehicleSet vehicle_set_;
  common::LaneNet lane_net_;
  common::ParkingSpots spots_;
  common::WaypointsGraph graph_;
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
  void GetLanRawFromRoute(const std::vector<std::pair<double, double>>& route,
                          common::LaneRaw* p_lane);
  common::WaypointsGraph GetGraphFromSimulationData(
      const std::unordered_map<std::string, Route>& routes);
  void GetParkingSpotsFromSimulationData(
      std::unordered_map<std::string, std::vector<ParkingSpot>>& sim_spots,
      common::ParkingSpots* parking_spots);
  void updateMap(Environment* env, ParkingMap* parkingMap);
};
}  // namespace semantic_map_manager