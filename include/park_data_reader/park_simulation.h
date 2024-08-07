#ifndef PARKSIMULATION_HPP
#define PARKSIMULATION_HPP

#include <string>

#include "park_data_reader/loader.h"
#include "park_data_reader/park_env.h"
namespace park {
class ParkSimulation {
 public:
  ParkSimulation(const std::string& sceneFile, const std::string& agentsFile,
                 const std::string& framesFile,
                 const std::string& instancesFile,
                 const std::string& obstaclesFile, int startFrameIndex,
                 int endFrameIndex);

  void run();

 private:
  void loadData();
  void findStartFrame();

  std::string sceneFile;
  std::string agentsFile;
  std::string framesFile;
  std::string instancesFile;
  std::string obstaclesFile;
  int startFrameIndex;
  int endFrameIndex;

  Scene scene;
  std::unordered_map<std::string, Agent> agents;
  std::unordered_map<std::string, Frame> frames;
  std::unordered_map<std::string, Instance> instances;
  std::unordered_map<std::string, Obstacle> obstacles;

  Environment* env;
  std::string currentFrameToken;
};

}  // namespace park

#endif  // PARKSIMULATION_HPP
