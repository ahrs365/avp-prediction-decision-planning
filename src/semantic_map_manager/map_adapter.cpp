#include <iostream>
#include <thread>

#include "semantic_map_manager/map_adapter.h"
namespace semantic_map_manager {

MapAdapter::MapAdapter(SemanticMapManager* ptr_smm) {
  p_smm_ = ptr_smm;
  p_data_renderer_ = new DataRenderer(ptr_smm);
}
MapAdapter::~MapAdapter() {}
void MapAdapter::run(double cycle_time_ms) {
  const size_t max_queue_size = 10;  // 设置队列的最大容量

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
      if (envQueue_->size() > 1) {
        envQueue_->pop();
      }
      if (parkingMapQueue_->size() > 1) {
        parkingMapQueue_->pop();
      }
    }

    // 生成语义地图
    updateMap(env, parkingMap);
    {
      std::lock_guard<std::mutex> lock(*mutex_);
      if (smmQueue_->size() >= max_queue_size) {
        smmQueue_->pop();  // 删除最早的元素
      }
      smmQueue_->push(p_smm_);
    }
    cv_->notify_all();

    // 控制建图线程周期
    std::this_thread::sleep_until(
        start_time +
        std::chrono::milliseconds(static_cast<int>(cycle_time_ms)));
    auto end_time = std::chrono::steady_clock::now();
    std::cout << "MapAdapter::run() cycle time: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(
                     end_time - start_time)
                     .count()
              << "ms" << std::endl;
  }
}

void semantic_map_manager::MapAdapter::setQueues(
    std::queue<Environment*>& envQueue,
    std::queue<ParkingMap*>& parkingMapQueue,
    std::queue<SemanticMapManager*>& smmQueue, std::mutex& mutex,
    std::condition_variable& cv) {
  envQueue_ = &envQueue;
  parkingMapQueue_ = &parkingMapQueue;
  smmQueue_ = &smmQueue;
  mutex_ = &mutex;
  cv_ = &cv;
}

void MapAdapter::GetSimulationDataFromStatic(ParkingMap* parkingMap) {
  get_arena_info_static_ = true;
}

void MapAdapter::GetSimulationDataFromDynamic(Environment* env) {
  time_stamp_ += 0.01;
}

void MapAdapter::updateMap(Environment* env, ParkingMap* parkingMap) {
  if (!get_arena_info_static_) {
    // 静态障碍物转换(一次就行)
    GetSimulationDataFromStatic(parkingMap);
  } else {
    // 动态障碍物转换
    GetSimulationDataFromDynamic(env);
  }
  p_data_renderer_->Render(time_stamp_, lane_net_, vehicle_set_, obstacle_set_);
}
}  // namespace semantic_map_manager