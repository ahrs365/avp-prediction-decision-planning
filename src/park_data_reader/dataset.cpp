#include <Python.h>

#include <fstream>
#include <iostream>
#include <stdexcept>

#include "park_data_reader/dataset.h"

nlohmann::json Frame::to_json() const {
  return {{"token", token},
          {"timestamp", timestamp},
          {"next", next},
          {"prev", prev},
          {"instances", instances}};
}

nlohmann::json Agent::to_json() const {
  return {{"token", token},
          {"first_instance", first_instance},
          {"type", type},
          {"size", size}};
}

nlohmann::json Instance::to_json() const {
  return {{"token", token},
          {"coords", coords},
          {"heading", heading},
          {"speed", speed},
          {"next", next},
          {"prev", prev},
          {"agent_token", agent_token}};
}

nlohmann::json Obstacle::to_json() const {
  return {{"token", token},
          {"coords", coords},
          {"size", size},
          {"heading", heading}};
}

nlohmann::json Scene::to_json() const {
  return {{"scene_token", scene_token},
          {"first_frame", first_frame},
          {"obstacles", obstacles}};
}

void Dataset::load_from_python(const std::string& script,
                               const std::string& function,
                               const std::string& filename) {
  py::scoped_interpreter guard{};
  py::module_ sys = py::module_::import("sys");
  sys.attr("path").attr("append")(
      "/home/ahrs/workspace/nday/bspline_lattice_planner/scripts");
  py::module_ dataset_module = py::module_::import(script.c_str());
  py::object get_data = dataset_module.attr(function.c_str());

  std::string data = get_data(filename).cast<std::string>();
  auto json_data = nlohmann::json::parse(data);

  for (const auto& item : json_data["frames"].items()) {
    Frame frame;
    frame.token = item.key();
    frame.timestamp = item.value().at("timestamp").get<double>();
    frame.next = item.value().at("next").get<std::string>();
    frame.prev = item.value().at("prev").get<std::string>();
    frame.instances =
        item.value().at("instances").get<std::vector<std::string>>();
    frames[frame.token] = frame;
  }

  for (const auto& item : json_data["agents"].items()) {
    Agent agent;
    agent.token = item.key();
    agent.first_instance = item.value().at("first_instance").get<std::string>();
    agent.type = item.value().at("type").get<std::string>();
    agent.size = item.value().at("size").get<std::vector<double>>();
    agents[agent.token] = agent;
  }

  for (const auto& item : json_data["instances"].items()) {
    Instance instance;
    instance.token = item.key();
    instance.coords = item.value().at("coords").get<std::vector<double>>();
    instance.heading = item.value().at("heading").get<double>();
    instance.speed = item.value().at("speed").get<double>();
    instance.next = item.value().at("next").get<std::string>();
    instance.prev = item.value().at("prev").get<std::string>();
    instance.agent_token = item.value().at("agent_token").get<std::string>();
    instances[instance.token] = instance;
  }

  for (const auto& item : json_data["obstacles"].items()) {
    Obstacle obstacle;
    obstacle.token = item.key();
    obstacle.coords = item.value().at("coords").get<std::vector<double>>();
    obstacle.size = item.value().at("size").get<std::vector<double>>();
    obstacle.heading = item.value().at("heading").get<double>();
    obstacles[obstacle.token] = obstacle;
  }

  for (const auto& item : json_data["scenes"].items()) {
    Scene scene;
    scene.scene_token = item.key();
    scene.first_frame = item.value().at("first_frame").get<std::string>();
    scene.obstacles =
        item.value().at("obstacles").get<std::vector<std::string>>();
    scenes[scene.scene_token] = scene;
  }
}

json Dataset::get(const std::string& obj_type, const std::string& token) {
  if (obj_type == "frame") {
    return frames.at(token).to_json();
  } else if (obj_type == "agent") {
    return agents.at(token).to_json();
  } else if (obj_type == "instance") {
    return instances.at(token).to_json();
  } else if (obj_type == "obstacle") {
    return obstacles.at(token).to_json();
  } else if (obj_type == "scene") {
    return scenes.at(token).to_json();
  } else {
    throw std::invalid_argument("Invalid object type");
  }
}

std::vector<std::string> Dataset::list_scenes() {
  std::vector<std::string> scene_tokens;
  for (const auto& scene : scenes) {
    scene_tokens.push_back(scene.first);
  }
  return scene_tokens;
}

nlohmann::json Dataset::getFutureFrames(const std::string& frame_token,
                                        int timesteps) {
  std::vector<nlohmann::json> frames_json =
      getTimeline("frame", "next", frame_token, timesteps);
  return frames_json;
}

std::vector<nlohmann::json> Dataset::getTimeline(const std::string& obj_type,
                                                 const std::string& direction,
                                                 const std::string& token,
                                                 int timesteps) {
  std::unordered_map<std::string, nlohmann::json> obj_dict;

  if (obj_type == "frame") {
    for (const auto& item : frames) {
      obj_dict[item.first] = item.second.to_json();
    }
  } else if (obj_type == "instance") {
    for (const auto& item : instances) {
      obj_dict[item.first] = item.second.to_json();
    }
  } else {
    throw std::invalid_argument("Invalid object type");
  }

  std::vector<nlohmann::json> timeline;
  nlohmann::json current_obj = obj_dict.at(token);
  timeline.push_back(current_obj);

  std::string next_token = current_obj[direction];
  for (int i = 0; i < timesteps; ++i) {
    if (next_token.empty()) {
      break;
    }
    nlohmann::json next_obj = obj_dict.at(next_token);
    timeline.push_back(next_obj);
    next_token = next_obj[direction];
  }

  if (direction == "prev") {
    std::reverse(timeline.begin(), timeline.end());
  }

  return timeline;
}
