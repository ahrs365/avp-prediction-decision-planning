#include <thread>

#include "map_adapter.h"
namespace semantic_map_manager {

MapAdapter::MapAdapter() {}
MapAdapter::~MapAdapter() {}
void semantic_map_manager::MapAdapter::run(double cycle_time_ms) {
  while (true) {
    auto start_time = std::chrono::steady_clock::now();

    Environment* env;
    ParkingMap* parkingMap;

    // 等待数据
    {
      std::unique_lock<std::mutex> lock(*mutex_);
      cv_->wait(lock, [&] {
        return !envQueue_->empty() && !parkingMapQueue_->empty();
      });
      env = envQueue_->front();
      parkingMap = parkingMapQueue_->front();
      envQueue_->pop();
      parkingMapQueue_->pop();
    }

    // 生成语义地图
    SemanticMapManager smm = generateSMM(env, parkingMap);

    // 将SMM地图发送到规划线程
    {
      std::lock_guard<std::mutex> lock(*mutex_);
      smmQueue_->push(smm);
    }
    cv_->notify_all();

    // 控制建图线程周期
    std::this_thread::sleep_until(
        start_time +
        std::chrono::milliseconds(static_cast<int>(cycle_time_ms)));
  }
}

void semantic_map_manager::MapAdapter::setQueues(
    std::queue<Environment*>& envQueue,
    std::queue<ParkingMap*>& parkingMapQueue,
    std::queue<SemanticMapManager>& smmQueue, std::mutex& mutex,
    std::condition_variable& cv) {
  envQueue_ = &envQueue;
  parkingMapQueue_ = &parkingMapQueue;
  smmQueue_ = &smmQueue;
  mutex_ = &mutex;
  cv_ = &cv;
}

SemanticMapManager semantic_map_manager::MapAdapter::generateSMM(
    Environment* env, ParkingMap* parkingMap) {
  return SemanticMapManager();
}
}  // namespace semantic_map_manager