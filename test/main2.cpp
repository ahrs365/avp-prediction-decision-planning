#include <FL/Fl.H>
#include <FL/Fl_Window.H>

#include <thread>

#include "eudm_planner/eudm_server.h"
#include "park_data_reader/drawing_area.h"
#include "park_data_reader/park_simulation.h"
#include "semantic_map_manager/map_adapter.h"

int main() {
  std::queue<Environment*> envQueue;
  std::queue<ParkingMap*> parkingMapQueue;
  std::queue<SemanticMapManager*> smmQueue;
  std::mutex mutex;
  std::condition_variable cv;

  // 创建对象
  park::ParkSimulation simulation(
      "./data/DJI_0012_scene.json", "./data/DJI_0012_agents.json",
      "./data/DJI_0012_frames.json", "./data/DJI_0012_instances.json",
      "./data/DJI_0012_obstacles.json", "./data/parking_map.yml", 0,
      100  // startFrameIndex, endFrameIndex
  );
  semantic_map_manager::SemanticMapManager semantic_map_manager;
  semantic_map_manager::MapAdapter mapAdapter(&semantic_map_manager);
  planning::EudmServer planner;

  // 设置队列
  simulation.setQueues(envQueue, parkingMapQueue, mutex, cv);
  mapAdapter.setQueues(envQueue, parkingMapQueue, smmQueue, mutex, cv);
  planner.setQueue(smmQueue, mutex, cv);

  // 启动线程，设置不同的周期
  double replay_cycle_time_ms = 100.0;  // 回放线程周期
  double map_cycle_time_ms = 200.0;     // 建图线程周期
  double planner_cycle_time_ms = 50.0;  // 规划线程周期

  // 修改后的代码
  std::thread replayThread(&park::ParkSimulation::run, &simulation,
                           replay_cycle_time_ms);
  std::thread mapThread(&semantic_map_manager::MapAdapter::run, &mapAdapter,
                        map_cycle_time_ms);
  std::thread plannerThread(&planning::EudmServer::run, &planner,
                            planner_cycle_time_ms);

  // 等待线程结束
  replayThread.join();
  mapThread.join();
  plannerThread.join();

  return 0;
}