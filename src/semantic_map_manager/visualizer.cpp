#include "semantic_map_manager/visualizer.h"

namespace semantic_map_manager {
Visualizer::Visualizer(int node_id) : node_id_(node_id) {
  std::cout << "node_id_ = " << node_id_ << std::endl;
}
Visualizer::Visualizer() {}
Visualizer::~Visualizer() {}
void Visualizer::VisualizeDataWithStamp(const double& stamp,
                                        const SemanticMapManager& smm) {
  VisualizeEgoVehicle(stamp, smm.ego_vehicle());
  VisualizeObstacleMap(stamp, smm.obstacle_map());
  VisualizeSurroundingLaneNet(stamp, smm.surrounding_lane_net(),
                              std::vector<int>());
  VisualizeSurroundingVehicles(stamp, smm.surrounding_vehicles(),
                               smm.key_vehicle_ids());
}

void Visualizer::VisualizeGraph(const double& stamp,
                                const common::WaypointsGraph& way_graph) {}
void Visualizer::VisualizeEgoVehicle(const double& stamp,
                                     const common::Vehicle& vehicle) {}
void Visualizer::VisualizeSurroundingLaneNet(
    const double& stamp, const common::LaneNet& lane_net,
    const std::vector<int>& deleted_lane_ids) {}
void Visualizer::VisualizeSurroundingVehicles(
    const double& stamp, const common::VehicleSet& vehicle_set,
    const std::vector<int>& nearby_ids) {}
void Visualizer::VisualizeObstacleMap(
    const double& stamp, const common::GridMapND<uint8_t, 2>& obstacle_map) {}
}  // namespace semantic_map_manager
