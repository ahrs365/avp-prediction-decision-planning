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

  // route
  auto route = env->getParkingMap().getRoutes();
  GetGraphFromSimulationData(route, &graph_);

  // parking spots
  auto parking_spots = env->getParkingMap().getParkingSpots();
  GetParkingSpotsFromSimulationData(parking_spots, &spots_);
}

void MapAdapter::GetSimulationDataFromDynamic(Environment* env) {
  printf("[MapAdapter] GetSimulationDataFromDynamic ...\n");
  time_stamp_ = env->getTimeStamp();
  int id = 1;
  //这里注意不要用 const auto& obstacle =
  // env->getCurrentDynamicObstacles(),避免线程之间冲突
  auto obstacles = env->getCurrentDynamicObstacles();

  for (const auto obs : obstacles) {
    common::Vehicle vehicle;
    vehicle.set_id(id++);
    GetVehicleFromSimulationData(obs.second, &vehicle);
    vehicle_set_.vehicles.insert(
        std::pair<int, common::Vehicle>(vehicle.id(), vehicle));
  }

  //设置自车起始位置
  common::Vehicle ego_vehicle;
  common::State state;
  state.vec_position = {3.07, 8};
  state.angle = M_PI_2;
  ego_vehicle.set_state(state);

  vehicle_set_.vehicles.insert(std::pair<int, common::Vehicle>(0, ego_vehicle));
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

void MapAdapter::GetLanRawFromRoute(
    const std::vector<std::pair<double, double>>& route,
    common::LaneRaw* p_lane) {
  if (route.empty()) {
    return;
  }
  p_lane->id = 0;
  p_lane->dir = 0;
  p_lane->child_id = {};
  p_lane->father_id = {};
  p_lane->l_lane_id = 0;
  p_lane->l_change_avbl = false;
  p_lane->r_lane_id = 0;
  p_lane->r_change_avbl = false;
  p_lane->behavior = "";
  p_lane->length = 0;

  p_lane->start_point(0) = route.front().first;
  p_lane->start_point(1) = route.front().second;
  p_lane->final_point(0) = route.back().first;
  p_lane->final_point(1) = route.back().second;
  for (const auto pt : route) {
    p_lane->lane_points.push_back(Vec2f(pt.first, pt.second));
  }
}

void MapAdapter::GetGraphFromSimulationData(
    const std::unordered_map<std::string, Route>& routes,
    common::WaypointsGraph* p_graph) {
  // 为每个路线添加航路点
  for (const auto& route : routes) {
    p_graph->add_waypoint_list(route.second.points);
  }

  // 连接指定的航路点
  // Connect sections
  p_graph->connect(routes.at("C2").points[3], routes.at("R2L").points[0]);
  p_graph->connect(routes.at("C2").points[7], routes.at("R3L").points[0]);

  p_graph->connect(routes.at("C2").points[3], routes.at("R2R").points.back());
  p_graph->connect(routes.at("C2").points[7], routes.at("R3R").points.back());

  p_graph->connect(routes.at("R1L").points[0], routes.at("R1R").points.back());

  // Connect corners
  p_graph->connect(routes.at("C1").points[0], routes.at("R1C1BR").points[0]);
  p_graph->connect(routes.at("R1C1BR").points.back(),
                   routes.at("R1L").points.back());
  p_graph->connect(routes.at("C1").points[2], routes.at("R2C1TR").points[0]);
  p_graph->connect(routes.at("R2C1TR").points.back(),
                   routes.at("R2L").points.back());
  p_graph->connect(routes.at("C1").points[4], routes.at("R2C1BR").points[0]);
  p_graph->connect(routes.at("R2C1BR").points.back(),
                   routes.at("R2L").points.back());
  p_graph->connect(routes.at("C1").points[6], routes.at("R3C1TR").points[0]);
  p_graph->connect(routes.at("R3C1TR").points.back(),
                   routes.at("R3L").points.back());
  p_graph->connect(routes.at("C1").points[8], routes.at("R3C1BR").points[0]);
  p_graph->connect(routes.at("R3C1BR").points.back(),
                   routes.at("R3L").points.back());
  p_graph->connect(routes.at("C1").points[10], routes.at("R4C1TR").points[0]);
  p_graph->connect(routes.at("R4C1TR").points.back(),
                   routes.at("R4L").points.back());
  p_graph->connect(routes.at("C1").points[12], routes.at("R4C1BR").points[0]);
  p_graph->connect(routes.at("R4C1BR").points.back(),
                   routes.at("R4L").points.back());

  p_graph->connect(routes.at("C2").points[0], routes.at("R1C2BL").points[0]);
  p_graph->connect(routes.at("R1C2BL").points.back(),
                   routes.at("R1L").points[2]);
  p_graph->connect(routes.at("C2").points[2], routes.at("R2C2TL").points[0]);
  p_graph->connect(routes.at("R2C2TL").points.back(),
                   routes.at("R2L").points[0]);
  p_graph->connect(routes.at("C2").points[4], routes.at("R2C2BL").points[0]);
  p_graph->connect(routes.at("R2C2BL").points.back(),
                   routes.at("R2L").points[0]);
  p_graph->connect(routes.at("C2").points[6], routes.at("R3C2TL").points[0]);
  p_graph->connect(routes.at("R3C2TL").points.back(),
                   routes.at("R3L").points[0]);
  p_graph->connect(routes.at("C2").points[8], routes.at("R3C2BL").points[0]);
  p_graph->connect(routes.at("R3C2BL").points.back(),
                   routes.at("R3L").points[0]);
  p_graph->connect(routes.at("C2").points[10], routes.at("R4C2TL").points[0]);
  p_graph->connect(routes.at("R4C2TL").points.back(),
                   routes.at("R4L").points[0]);
  p_graph->connect(routes.at("C2").points[12], routes.at("R4C2BL").points[0]);
  p_graph->connect(routes.at("R4C2BL").points.back(),
                   routes.at("R4L").points[0]);

  p_graph->connect(routes.at("C2").points[0], routes.at("R1C2BR").points[0]);
  p_graph->connect(routes.at("R1C2BR").points.back(),
                   routes.at("R1R").points.back());
  p_graph->connect(routes.at("C2").points[2], routes.at("R2C2TR").points[0]);
  p_graph->connect(routes.at("R2C2TR").points.back(),
                   routes.at("R2R").points.back());
  p_graph->connect(routes.at("C2").points[4], routes.at("R2C2BR").points[0]);
  p_graph->connect(routes.at("R2C2BR").points.back(),
                   routes.at("R2R").points.back());
  p_graph->connect(routes.at("C2").points[6], routes.at("R3C2TR").points[0]);
  p_graph->connect(routes.at("R3C2TR").points.back(),
                   routes.at("R3R").points.back());
  p_graph->connect(routes.at("C2").points[8], routes.at("R3C2BR").points[0]);
  p_graph->connect(routes.at("R3C2BR").points.back(),
                   routes.at("R3R").points.back());
  p_graph->connect(routes.at("C2").points[10], routes.at("R4C2TR").points[0]);
  p_graph->connect(routes.at("R4C2TR").points.back(),
                   routes.at("R4R").points.back());
  p_graph->connect(routes.at("C2").points[12], routes.at("R4C2BR").points[0]);
  p_graph->connect(routes.at("R4C2BR").points.back(),
                   routes.at("R4R").points.back());

  p_graph->connect(routes.at("R4L").points[0], routes.at("R4R").points.back());

  p_graph->connect(routes.at("EXT").points[1], routes.at("EXTL").points[0]);
  p_graph->connect(routes.at("EXT").points[1], routes.at("EXTR").points[0]);
  p_graph->connect(routes.at("EXTL").points[0], routes.at("R1L").points.back());
  p_graph->connect(routes.at("EXTR").points[0],
                   routes.at("R1L").points[routes.at("R1L").points.size() - 5]);

  // 打印图信息
  // p_graph->print_graph();

  // 定义起点和终点
  common::Vertex* start = p_graph->search({80.45, 64.95});
  common::Vertex* goal = p_graph->search({3.07, 4.5});

  // 运行 A* 算法
  common::AStarPlanner planner(start, goal);
  common::AStarGraph result = planner.solve();
  auto ref_path = result.compute_ref_path();

  common::LaneRaw lane_raw;
  lane_raw.id = 0;
  GetLanRawFromRoute(ref_path, &lane_raw);
  lane_net_.lane_set.insert(
      std::pair<int, common::LaneRaw>(lane_raw.id, lane_raw));
}

void MapAdapter::GetParkingSpotsFromSimulationData(
    std::unordered_map<std::string, std::vector<ParkingSpot>>& sim_spots,
    common::ParkingSpots* parking_spots) {
  for (auto area : sim_spots) {
    for (auto spot : area.second) {
      common::Spot new_spot;
      new_spot.id = spot.id;
      new_spot.type = 0;
      for (auto p : spot.corners) {
        common::Point pt;
        pt.x = p.first;
        pt.y = p.second;
        new_spot.polygon.points.push_back(pt);
      }
      parking_spots->spot_polygon.insert(
          std::pair<int, common::Spot>(new_spot.id, new_spot));
    }
  }
}

void MapAdapter::updateMap(Environment* env, ParkingMap* parkingMap) {
  printf("[MapAdapter] updateMap ...\n");
  if (!get_arena_info_static_) {
    // 静态障碍物转换(一次就行)
    GetSimulationDataFromStatic(env);

  } else {
    // 动态障碍物转换
    GetSimulationDataFromDynamic(env);
    p_data_renderer_->Render(time_stamp_, lane_net_, vehicle_set_,
                             obstacle_set_, graph_);
  }
}
}  // namespace semantic_map_manager