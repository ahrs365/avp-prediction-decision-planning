#include <FL/Fl.H>
#include <FL/Fl_Window.H>

#include "drawing_area.h"
#include "park_simulation.h"

int main() {
  Fl_Window window(1500, 1000, "FLTK Park Simulation");
  // 创建绘图区域，初始大小为窗口大小减去边框区域
  park::DrawingArea drawingArea(10, 10, window.w() - 20, window.h() - 20);

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