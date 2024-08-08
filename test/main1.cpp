#include <iostream>

#include "park_data_reader/park_simulation.h"
#include "park_data_reader/parking_map.h"
int main() {
  park::ParkSimulation simulation(
      "./data/DJI_0012_scene.json", "./data/DJI_0012_agents.json",
      "./data/DJI_0012_frames.json", "./data/DJI_0012_instances.json",
      "./data/DJI_0012_obstacles.json", "./data/parking_map.yml", 0,
      100  // startFrameIndex, endFrameIndex
  );

  simulation.run();

  return 0;
}
