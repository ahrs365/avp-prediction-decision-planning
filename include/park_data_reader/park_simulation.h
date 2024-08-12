#ifndef PARKSIMULATION_HPP
#define PARKSIMULATION_HPP

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <string>
#include <unordered_map>

#include "drawing_area.h"
#include "park_data_reader/loader.h"
#include "park_data_reader/park_env.h"
#include "park_data_reader/parking_map.h"
namespace park {
class ParkSimulation {
 public:
  ParkSimulation(const std::string& sceneFile, const std::string& agentsFile,
                 const std::string& framesFile,
                 const std::string& instancesFile,
                 const std::string& obstaclesFile, const std::string& mapFile,
                 int startFrameIndex, int endFrameIndex);

  void run(double cycle_time_ms);
  void setQueues(std::queue<Environment*>& envQueue,
                 std::queue<ParkingMap*>& parkingMapQueue, std::mutex& mutex,
                 std::condition_variable& cv);

 private:
  void loadData();
  void findStartFrame();
  void drawStaticElements();
  void drawFrame(const Frame& frame);

  std::string sceneFile;
  std::string agentsFile;
  std::string framesFile;
  std::string instancesFile;
  std::string obstaclesFile;
  std::string mapFile;
  int startFrameIndex;
  int endFrameIndex;

  Scene scene;
  std::unordered_map<std::string, Agent> agents;
  std::unordered_map<std::string, Frame> frames;
  std::unordered_map<std::string, Instance> instances;
  std::unordered_map<std::string, Obstacle> obstacles;
  ParkingMap parkingMap;
  bool staticElementsDrawn = false;
  Environment* env;
  std::string currentFrameToken;
  std::queue<Environment*>* envQueue_;
  std::queue<ParkingMap*>* parkingMapQueue_;
  std::mutex* mutex_;
  std::condition_variable* cv_;
};

}  // namespace park

#endif  // PARKSIMULATION_HPP
