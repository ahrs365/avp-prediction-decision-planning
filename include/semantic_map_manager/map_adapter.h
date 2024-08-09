#pragma once
#include <condition_variable>
#include <mutex>
#include <queue>

#include "park_data_reader/park_simulation.h"
#include "semantic_map_manager/semantic_map_manager.h"

using namespace park;

namespace semantic_map_manager {
class MapAdapter {
 public:
  MapAdapter();
  ~MapAdapter();
  void run(double cycle_time_ms);

  void setQueues(std::queue<Environment*>& envQueue,
                 std::queue<ParkingMap*>& parkingMapQueue,
                 std::queue<SemanticMapManager>& smmQueue, std::mutex& mutex,
                 std::condition_variable& cv);

 private:
  std::queue<Environment*>* envQueue_;
  std::queue<ParkingMap*>* parkingMapQueue_;
  std::queue<SemanticMapManager>* smmQueue_;
  std::mutex* mutex_;
  std::condition_variable* cv_;

  SemanticMapManager generateSMM(Environment* env, ParkingMap* parkingMap);
};
}  // namespace semantic_map_manager