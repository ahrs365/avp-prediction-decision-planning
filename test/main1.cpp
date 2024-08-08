#include <iostream>

#include "park_data_reader/park_simulation.h"

int main() {
  park::ParkSimulation simulation(
      "./data/DJI_0002_scene.json", "./data/DJI_0002_agents.json",
      "./data/DJI_0002_frames.json", "./data/DJI_0002_instances.json",
      "./data/DJI_0002_obstacles.json", "./data/parking_map.yml", 0,
      10  // startFrameIndex, endFrameIndex
  );

  simulation.run();

  return 0;
}
