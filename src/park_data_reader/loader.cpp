#include <fstream>
#include <iostream>

#include "park_data_reader/loader.h"
#include "yaml-cpp/yaml.h"  // yaml-cpp

namespace park {

std::unordered_map<std::string, Agent> Agent::loadFromFile(
    const std::string& filepath) {
  std::ifstream file(filepath);
  if (!file.is_open()) {
    std::cerr << "Error: Could not open file " << filepath << std::endl;
    return {};
  }
  json j;
  file >> j;

  std::unordered_map<std::string, Agent> agentsMap;
  for (auto& [key, value] : j.items()) {
    Agent agent = {key,
                   value["scene_token"],
                   value["type"],
                   value["size"].get<std::vector<double>>(),
                   value["first_instance"],
                   value["last_instance"]};
    agentsMap[key] = agent;
  }
  return agentsMap;
}

std::unordered_map<std::string, Instance> Instance::loadFromFile(
    const std::string& filepath,
    const std::unordered_map<std::string, Frame>& frames) {
  std::ifstream file(filepath);
  if (!file.is_open()) {
    std::cerr << "Error: Could not open file " << filepath << std::endl;
    return {};
  }
  json j;
  file >> j;

  std::unordered_map<std::string, Instance> instancesMap;
  for (auto& value : j) {
    std::string frame_token = value["frame_token"];
    if (frames.find(frame_token) != frames.end()) {
      std::string key = value["instance_token"];
      Instance instance = {key,
                           value["agent_token"],
                           value["frame_token"],
                           value["coords"].get<std::vector<double>>(),
                           value["heading"],
                           value["speed"],
                           value["acceleration"].get<std::vector<double>>(),
                           value["mode"],
                           value["prev"],
                           value["next"]};
      instancesMap[key] = instance;
    }
  }
  return instancesMap;
}

std::unordered_map<std::string, Obstacle> Obstacle::loadFromFile(
    const std::string& filepath,
    const std::unordered_map<std::string, Frame>& frames) {
  std::ifstream file(filepath);
  if (!file.is_open()) {
    std::cerr << "Error: Could not open file " << filepath << std::endl;
    return {};
  }
  json j;
  try {
    file >> j;
  } catch (const json::parse_error& e) {
    std::cerr << "Parse error: " << e.what() << std::endl;
    return {};
  }

  std::unordered_map<std::string, Obstacle> obstaclesMap;
  for (auto& value : j) {
    try {
      if (value["frame_token"].is_null()) {
        continue;
      }

      std::string frame_token = value["frame_token"];
      if (frames.find(frame_token) != frames.end()) {
        std::string key = value["obstacle_token"];
        Obstacle obstacle = {key,
                             value["scene_token"],
                             value["type"],
                             value["size"].get<std::vector<double>>(),
                             value["coords"].get<std::vector<double>>(),
                             value["heading"]};
        obstaclesMap[key] = obstacle;
      }
    } catch (const json::type_error& e) {
      std::cerr << "Type error in JSON data: " << e.what() << std::endl;
      std::cerr << "Offending JSON: " << value.dump() << std::endl;
    } catch (const std::exception& e) {
      std::cerr << "Error processing JSON data: " << e.what() << std::endl;
      std::cerr << "Offending JSON: " << value.dump() << std::endl;
    }
  }
  return obstaclesMap;
}

std::unordered_map<std::string, Frame> Frame::loadFromFile(
    const std::string& filepath, int start_index, int end_index) {
  std::ifstream file(filepath);
  if (!file.is_open()) {
    std::cerr << "Error: Could not open file " << filepath << std::endl;
    return {};
  }
  json j;
  file >> j;

  std::vector<std::pair<std::string, Frame>> framesVec;
  for (auto& [key, value] : j.items()) {
    Frame frame = {key,
                   value["scene_token"],
                   value["timestamp"],
                   value["prev"],
                   value["next"],
                   value["instances"].get<std::vector<std::string>>()};
    framesVec.emplace_back(key, frame);
  }

  // Sort frames by timestamp to ensure the order is from smallest to largest
  std::sort(framesVec.begin(), framesVec.end(),
            [](const auto& a, const auto& b) {
              return a.second.timestamp < b.second.timestamp;
            });

  std::unordered_map<std::string, Frame> framesMap;
  for (int i = start_index; i <= end_index && i < framesVec.size(); ++i) {
    framesMap[framesVec[i].first] = framesVec[i].second;
  }

  return framesMap;
}

Scene Scene::loadFromFile(const std::string& filepath) {
  std::ifstream file(filepath);
  if (!file.is_open()) {
    std::cerr << "Error: Could not open file " << filepath << std::endl;
    return {};
  }
  json j;
  file >> j;

  Scene scene = {j["scene_token"],
                 j["filename"],
                 j["timestamp"],
                 j["first_frame"],
                 j["last_frame"],
                 j["agents"].get<std::vector<std::string>>(),
                 j["obstacles"].get<std::vector<std::string>>()};
  return scene;
}

// 读取 parking_map.yml 并解析内容
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
    Waypoint wp;
    wp.start = {waypoint.second["bounds"][0][0].as<double>(),
                waypoint.second["bounds"][0][1].as<double>()};
    wp.end = {waypoint.second["bounds"][1][0].as<double>(),
              waypoint.second["bounds"][1][1].as<double>()};
    wp.nums = waypoint.second["nums"].as<int>();
    parkingMap.waypoints[waypoint.first.as<std::string>()] = wp;
  }

  return parkingMap;
}

}  // namespace park
