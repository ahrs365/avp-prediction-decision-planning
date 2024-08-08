#include <FL/Fl.H>
#include <FL/Fl_Window.H>

#include "drawing_area.h"
#include "park_simulation.h"

int main() {
  Fl_Window window(800, 600, "FLTK Park Simulation");
  park::DrawingArea drawingArea(10, 10, 780, 580);

  park::ParkSimulation simulation(
      "./data/DJI_0012_scene.json", "./data/DJI_0012_agents.json",
      "./data/DJI_0012_frames.json", "./data/DJI_0012_instances.json",
      "./data/DJI_0012_obstacles.json", "./data/parking_map.yml", 0,
      100  // startFrameIndex, endFrameIndex
  );

  window.end();
  window.show();

  simulation.run(&drawingArea);

  return Fl::run();
}