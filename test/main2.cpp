

#include "eudm_planner/eudm_server.h"
#include "park_data_reader/drawing_area.h"
#include "park_data_reader/park_simulation.h"
#include "semantic_map_manager/map_adapter.h"

int main() {
  // 创建对象
  park::ParkSimulation simulation(
      "./data/DJI_0012_scene.json", "./data/DJI_0012_agents.json",
      "./data/DJI_0012_frames.json", "./data/DJI_0012_instances.json",
      "./data/DJI_0012_obstacles.json", "./data/parking_map.yml", 0,
      100  // startFrameIndex, endFrameIndex
  );

  simulation.run();

  return 0;
}