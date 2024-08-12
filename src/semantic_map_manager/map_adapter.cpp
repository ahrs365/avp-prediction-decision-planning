#include <iostream>
#include <thread>

#include "semantic_map_manager/map_adapter.h"
namespace semantic_map_manager {

MapAdapter::MapAdapter(SemanticMapManager* ptr_smm) {
  printf("[MapAdapter] Initialization ...\n");
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

void MapAdapter::GetSimulationDataFromStatic(Environment* env) {
  printf("[MapAdapter] GetSimulationDataFromStatic ...\n");
  get_arena_info_static_ = true;
  // GetLaneNetFromRosLaneNet

  // GetObstacleSetFromRosObstacleSet
  obstacle_set_.obs_circle.clear();
  obstacle_set_.obs_polygon.clear();
  auto const& obstacles = env->getCurrentStaticObstacles();
  int id = 0;
  for (const auto obs : obstacles) {
    // common::CircleObstacle obs_temp;
    // GetCircleObstacleFromRosCircleObstacle(obs, &obs_temp);
    common::PolygonObstacle obs_temp;
    obs_temp.id = id++;
    obs_temp.type = 0;
    GetPolygonFromSimulationData(obs.second, &obs_temp);
    obstacle_set_.obs_polygon.insert(
        std::pair<int, common::PolygonObstacle>(obs_temp.id, obs_temp));
  }
}

void MapAdapter::GetSimulationDataFromDynamic(Environment* env) {
  printf("[MapAdapter] GetSimulationDataFromDynamic ...\n");
  time_stamp_ = env->getTimeStamp();
  int id = 0;
  //这里注意不要用 const auto& obstacle =
  //env->getCurrentDynamicObstacles(),避免线程之间冲突
  auto obstacles = env->getCurrentDynamicObstacles();

  for (const auto obs : obstacles) {
    common::Vehicle vehicle;
    vehicle.set_id(id++);
    GetVehicleFromSimulationData(obs.second, &vehicle);
    vehicle_set_.vehicles.insert(
        std::pair<int, common::Vehicle>(vehicle.id(), vehicle));
  }
}

void MapAdapter::GetPolygonFromSimulationData(const StaticObstacle& obs,
                                              common::PolygonObstacle* poly) {
  double theta = obs.heading;
  double half_length = obs.size[0] / 2;
  double half_width = obs.size[1] / 2;
  std::vector<std::pair<double, double>> corners = {
      {obs.coords[0] + half_length * std::cos(theta) -
           half_width * std::sin(theta),
       obs.coords[1] + half_length * std::sin(theta) +
           half_width * std::cos(theta)},
      {obs.coords[0] - half_length * std::cos(theta) -
           half_width * std::sin(theta),
       obs.coords[1] - half_length * std::sin(theta) +
           half_width * std::cos(theta)},
      {obs.coords[0] - half_length * std::cos(theta) +
           half_width * std::sin(theta),
       obs.coords[1] - half_length * std::sin(theta) -
           half_width * std::cos(theta)},
      {obs.coords[0] + half_length * std::cos(theta) +
           half_width * std::sin(theta),
       obs.coords[1] + half_length * std::sin(theta) -
           half_width * std::cos(theta)}};
  for (const auto p : corners) {
    common::Point pt;
    pt.x = p.first;
    pt.y = p.second;
    poly->polygon.points.push_back(pt);
  }
}
void MapAdapter::GetVehicleFromSimulationData(const DynamicObstacle& obs,
                                              common::Vehicle* vehicle) {
  vehicle->set_subclass(obs.type);
  vehicle->set_type("VehicleWithBicycleKinematics");
  common::VehicleParam param;
  param.set_width(obs.size[1]);
  param.set_length(obs.size[0]);
  param.set_wheel_base(2.85);
  param.set_front_suspension((obs.size[0] - 2.85) / 2);
  param.set_rear_suspension((obs.size[0] - 2.85) / 2);
  param.set_max_steering_angle(45);
  param.set_max_longitudinal_acc(2.0);
  param.set_max_lateral_acc(2.0);
  // param.set_d_cr(msg.d_cr);

  common::State state;
  // GetStateFromSimulationData(obs, &state);
  state.time_stamp = time_stamp_;
  state.vec_position = {obs.coords[0], obs.coords[1]};
  state.angle = obs.heading;
  state.curvature = 0;
  state.velocity = obs.speed;
  state.acceleration = std::sqrt(obs.acceleration[0] * obs.acceleration[0] +
                                 obs.acceleration[1] * obs.acceleration[1]);
  state.steer = 0;
  vehicle->set_param(param);
  vehicle->set_state(state);
}
void MapAdapter::updateMap(Environment* env, ParkingMap* parkingMap) {
  printf("[MapAdapter] updateMap ...\n");
  if (!get_arena_info_static_) {
    // 静态障碍物转换(一次就行)
    GetSimulationDataFromStatic(env);
  } else {
    // 动态障碍物转换
    GetSimulationDataFromDynamic(env);
  }
  // p_data_renderer_->Render(time_stamp_, lane_net_, vehicle_set_,
  // obstacle_set_);
}
}  // namespace semantic_map_manager