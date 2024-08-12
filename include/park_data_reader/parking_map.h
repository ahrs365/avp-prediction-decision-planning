#ifndef PARKINGMAP_HPP
#define PARKINGMAP_HPP

#include <yaml-cpp/yaml.h>

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace park {

struct ParkingSpot {
  int id;
  std::vector<std::pair<double, double>> corners;
};

struct Route {
  std::vector<std::pair<double, double>> points;
};

struct ParkingArea {
  std::vector<std::pair<double, double>> bounds;
  struct Area {
    std::vector<double> coords;
    std::pair<int, int> shape;
  };
  std::vector<Area> areas;
};

class ParkingMap {
 public:
  static ParkingMap loadFromFile(const std::string& filepath);

  std::pair<int, int> getMapSize() const;
  //多线程，get方法不要用const &
  const std::unordered_map<std::string, std::vector<ParkingSpot>>
  getParkingSpots() const;
  const std::vector<std::pair<double, double>> getAllRoutePoints() const;
  void printInfo() const;

 private:
  std::pair<int, int> map_size;
  std::unordered_map<std::string, ParkingArea> parking_areas;
  std::unordered_map<std::string, std::vector<ParkingSpot>> parking_spots;
  std::vector<std::pair<double, double>> all_route_points;
  std::unordered_map<std::string, Route> routes;

  void generateParkingSpots();
  void generateRoutes();
};

}  // namespace park

#endif  // PARKINGMAP_HPP
