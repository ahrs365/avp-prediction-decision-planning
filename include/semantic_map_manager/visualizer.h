#ifndef VISUALIZER_H
#define VISUALIZER_H
#include <assert.h>

#include <iostream>
#include <vector>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/state.h"
#include "common/visualization/common_visualization_util.h"
#include "semantic_map_manager/semantic_map_manager.h"

namespace semantic_map_manager {
class Visualizer {
 public:
  Visualizer(int node_id);
  Visualizer();

  ~Visualizer();

  void VisualizeDataWithStamp(const double &stamp,
                              const SemanticMapManager &smm);
  void VisualizeGraph(const double &stamp,
                      const common::WaypointsGraph &way_graph);
  void VisualizeEgoVehicle(const double &stamp, const common::Vehicle &vehicle);
  void VisualizeSurroundingLaneNet(const double &stamp,
                                   const common::LaneNet &lane_net,
                                   const std::vector<int> &deleted_lane_ids);

  void VisualizeSurroundingVehicles(const double &stamp,
                                    const common::VehicleSet &vehicle_set,
                                    const std::vector<int> &nearby_ids);

  void VisualizeObstacleMap(const double &stamp,
                            const common::GridMapND<uint8_t, 2> &obstacle_map);

 private:
  int node_id_;

};  // class Visualizer
}  // namespace semantic_map_manager

#endif