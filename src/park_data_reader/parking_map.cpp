#include <cmath>
#include <iostream>
#include <stdexcept>

#include "parking_map.h"

namespace park {

ParkingMap ParkingMap::loadFromFile(const std::string& filepath) {
  YAML::Node config = YAML::LoadFile(filepath);
  ParkingMap parkingMap;

  // 读取地图尺寸
  parkingMap.map_size = std::make_pair(config["MAP_SIZE"]["x"].as<int>(),
                                       config["MAP_SIZE"]["y"].as<int>());

  // 读取停车区域
  for (const auto& area : config["PARKING_AREAS"]) {
    ParkingArea parkingArea;
    for (const auto& bound : area.second["bounds"]) {
      parkingArea.bounds.push_back(
          {bound[0].as<double>(), bound[1].as<double>()});
    }

    for (const auto& a : area.second["areas"]) {
      std::vector<double> coords;
      if (a["coords"]) {
        for (const auto& coord : a["coords"]) {
          coords.push_back(coord.as<double>());
        }
      }
      parkingArea.areas.push_back(
          {coords, {a["shape"][0].as<int>(), a["shape"][1].as<int>()}});
    }
    parkingMap.parking_areas[area.first.as<std::string>()] = parkingArea;
  }

  // 读取航路点
  for (const auto& waypoint : config["WAYPOINTS"]) {
    Route route;
    route.points.push_back({waypoint.second["bounds"][0][0].as<double>(),
                            waypoint.second["bounds"][0][1].as<double>()});
    route.points.push_back({waypoint.second["bounds"][1][0].as<double>(),
                            waypoint.second["bounds"][1][1].as<double>()});
    parkingMap.routes[waypoint.first.as<std::string>()] = route;
  }

  parkingMap.generateParkingSpots();
  parkingMap.generateRoutes();

  return parkingMap;
}

std::pair<int, int> ParkingMap::getMapSize() const { return map_size; }

const std::unordered_map<std::string, std::vector<ParkingSpot>>&
ParkingMap::getParkingSpots() const {
  return parking_spots;
}

const std::vector<std::pair<double, double>>& ParkingMap::getAllRoutePoints()
    const {
  return all_route_points;
}

void ParkingMap::generateParkingSpots() {
  int id = 1;
  for (const auto& area : parking_areas) {
    std::vector<ParkingSpot> spots;
    for (const auto& a : area.second.areas) {
      int rows = a.shape.first;
      int cols = a.shape.second;
      std::vector<std::pair<double, double>> bounds = area.second.bounds;

      double left_x_increment = (bounds[3].first - bounds[0].first) / rows;
      double left_y_increment = (bounds[3].second - bounds[0].second) / rows;
      double right_x_increment = (bounds[2].first - bounds[1].first) / rows;
      double right_y_increment = (bounds[2].second - bounds[1].second) / rows;

      for (int i = 0; i < rows; ++i) {
        std::vector<std::pair<double, double>> row_bounds = {
            {bounds[0].first + i * left_x_increment,
             bounds[0].second + i * left_y_increment},
            {bounds[1].first + i * right_x_increment,
             bounds[1].second + i * right_y_increment},
            {bounds[1].first + (i + 1) * right_x_increment,
             bounds[1].second + (i + 1) * right_y_increment},
            {bounds[0].first + (i + 1) * left_x_increment,
             bounds[0].second + (i + 1) * left_y_increment}};

        double row_top_x_increment =
            (row_bounds[1].first - row_bounds[0].first) / cols;
        double row_top_y_increment =
            (row_bounds[1].second - row_bounds[0].second) / cols;
        double row_bottom_x_increment =
            (row_bounds[2].first - row_bounds[3].first) / cols;
        double row_bottom_y_increment =
            (row_bounds[2].second - row_bounds[3].second) / cols;

        for (int j = 0; j < cols; ++j) {
          ParkingSpot spot;
          spot.id = id++;
          spot.corners = {
              {row_bounds[0].first + j * row_top_x_increment,
               row_bounds[0].second + j * row_top_y_increment},
              {row_bounds[0].first + (j + 1) * row_top_x_increment,
               row_bounds[0].second + (j + 1) * row_top_y_increment},
              {row_bounds[3].first + (j + 1) * row_bottom_x_increment,
               row_bounds[3].second + (j + 1) * row_bottom_y_increment},
              {row_bounds[3].first + j * row_bottom_x_increment,
               row_bounds[3].second + j * row_bottom_y_increment}};
          spots.push_back(spot);
        }
      }
    }
    parking_spots[area.first] = spots;
  }
}

void ParkingMap::generateRoutes() {
  all_route_points.clear();
  for (auto& route : routes) {
    auto& points = route.second.points;

    if (points.size() == 1) {
      // 只有一个点时，保留原始点
      all_route_points.push_back(points[0]);
      continue;
    }

    auto start = points[0];
    auto end = points[1];
    int num_points = std::ceil(
        std::hypot(end.first - start.first, end.second - start.second));

    if (num_points == 0) {
      // 如果起点和终点相同，则直接保留起点
      num_points = 1;
    }

    for (int i = 0; i <= num_points; ++i) {
      double t = static_cast<double>(i) / num_points;
      double x = (1 - t) * start.first + t * end.first;
      double y = (1 - t) * start.second + t * end.second;
      all_route_points.push_back({x, y});
    }
  }
}

void ParkingMap::printInfo() const {
  // 输出地图大小
  std::cout << "Map size: " << map_size.first << " x " << map_size.second
            << std::endl;

  // 输出停车位
  for (const auto& area : parking_spots) {
    std::cout << "Parking area: " << area.first << std::endl;
    for (const auto& spot : area.second) {
      std::cout << "  Parking spot ID: " << spot.id << ", ";
      for (const auto& corner : spot.corners) {
        std::cout << "    Corner: (" << corner.first << ", " << corner.second
                  << ")";
      }
      std::cout << std::endl;
    }
  }

  // 输出所有离散化后的航路点
  // std::cout << "All route points:" << std::endl;
  // for (const auto& point : all_route_points) {
  //   std::cout << "  Point: (" << point.first << ", " << point.second <<
  //   ")";
  // }
}

}  // namespace park
