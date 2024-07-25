#ifndef DATASET_H
#define DATASET_H

#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>
#include <vector>

using json = nlohmann::json;
namespace py = pybind11;

struct Frame {
  std::string token;
  double timestamp;
  std::string next;
  std::string prev;
  std::vector<std::string> instances;
  nlohmann::json to_json() const;
};

struct Agent {
  std::string token;
  std::string first_instance;
  std::string type;
  std::vector<double> size;
  nlohmann::json to_json() const;
};

struct Instance {
  std::string token;
  std::vector<double> coords;
  double heading;
  double speed;
  std::string next;
  std::string prev;
  std::string agent_token;
  nlohmann::json to_json() const;
};

struct Obstacle {
  std::string token;
  std::vector<double> coords;
  std::vector<double> size;
  double heading;
  nlohmann::json to_json() const;
};

struct Scene {
  std::string scene_token;
  std::string first_frame;
  std::vector<std::string> obstacles;
  nlohmann::json to_json() const;
};

struct ParkingArea {
  std::vector<Eigen::Vector2d> bounds;
  struct Area {
    std::vector<Eigen::Vector2d> coords;
    Eigen::Vector2i shape;
  };
  std::vector<Area> areas;
};

struct Waypoint {
  std::vector<Eigen::Vector2d> bounds;
  int nums;
};

class Dataset {
 public:
  std::unordered_map<std::string, Frame> frames;
  std::unordered_map<std::string, Agent> agents;
  std::unordered_map<std::string, Instance> instances;
  std::unordered_map<std::string, Obstacle> obstacles;
  std::unordered_map<std::string, Scene> scenes;

  void load_from_python(const std::string& script, const std::string& function,
                        const std::string& filename);
  json get(const std::string& obj_type, const std::string& token);
  std::vector<std::string> list_scenes();
  nlohmann::json getFutureFrames(const std::string& frame_token, int timesteps);
  std::vector<nlohmann::json> getTimeline(const std::string& obj_type,
                                          const std::string& direction,
                                          const std::string& token,
                                          int timesteps);
};

#endif  // DATASET_H
