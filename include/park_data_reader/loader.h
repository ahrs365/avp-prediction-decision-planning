#ifndef LOADER_HPP
#define LOADER_HPP

#include <string>
#include <unordered_map>
#include <vector>

#include "nlohmann/json.hpp"  // nlohmann/json

using json = nlohmann::json;

namespace park {
// 数据结构定义
struct Agent {
  std::string agent_token;
  std::string scene_token;
  std::string type;
  std::vector<double> size;
  std::string first_instance;
  std::string last_instance;

  static std::unordered_map<std::string, Agent> loadFromFile(
      const std::string& filepath);
};

struct Frame {
  std::string frame_token;
  std::string scene_token;
  double timestamp;
  std::string prev;
  std::string next;
  std::vector<std::string> instances;

  static std::unordered_map<std::string, Frame> loadFromFile(
      const std::string& filepath, int start_index, int end_index);
};

struct Instance {
  std::string instance_token;
  std::string agent_token;
  std::string frame_token;
  std::vector<double> coords;
  double heading;
  double speed;
  std::vector<double> acceleration;
  std::string mode;
  std::string prev;
  std::string next;

  static std::unordered_map<std::string, Instance> loadFromFile(
      const std::string& filepath,
      const std::unordered_map<std::string, Frame>& frames);
};

struct Obstacle {
  std::string obstacle_token;
  std::string scene_token;
  std::string type;
  std::vector<double> size;
  std::vector<double> coords;
  double heading;

  static std::unordered_map<std::string, Obstacle> loadFromFile(
      const std::string& filepath,
      const std::unordered_map<std::string, Frame>& frames);
};

struct Scene {
  std::string scene_token;
  std::string filename;
  std::string timestamp;
  std::string first_frame;
  std::string last_frame;
  std::vector<std::string> agents;
  std::vector<std::string> obstacles;

  static Scene loadFromFile(const std::string& filepath);
};

}  // namespace park
#endif  // LOADER_HPP
