#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <iostream>
#include <nlohmann/json.hpp>

#include "park_data_reader/dataset.h"
namespace py = pybind11;

int main() {
  Dataset dataset;
  dataset.load_from_python("dataset", "get_dataset_data", "./data/DJI_0012");

  // 示例: 打印一些加载的数据
  for (const auto& [token, frame] : dataset.frames) {
    std::cout << "Frame token: " << token << ", timestamp: " << frame.timestamp
              << std::endl;
  }

  // Get the first scene
  auto scenes = dataset.list_scenes();
  if (scenes.empty()) {
    std::cerr << "No scenes found in the dataset" << std::endl;
    return -1;
  }
  auto scene = dataset.get("scene", scenes[0]);

  // Get future frames
  auto frames = dataset.getFutureFrames(scene["first_frame"], 1000);

  return 0;
}
