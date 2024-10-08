
# 指定CMake的最小版本要求，确保使用的CMake具有我们需要的功能。
cmake_minimum_required(VERSION 3.12)

# 定义项目名称和版本，同时指定项目使用的编程语言。
project(avp VERSION 1.0.0 LANGUAGES CXX)

# 设置可执行文件和库文件的输出目录，以便统一管理构建产物。
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# 设置C++标准为C++11，并确保不使用特定编译器的扩展，增加代码的移植性。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# 允许通过命令行或GUI选项指定构建类型（例如Debug或Release）。
# 默认为Debug模式。
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

# 查找Python3和相关组件，包括NumPy，确保项目可以链接和使用这些库。
find_package(Python3 3.8 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(Eigen3 REQUIRED)
# 添加nlohmann/json库
find_package(nlohmann_json REQUIRED)
find_package(pybind11 REQUIRED)
add_subdirectory(third_party/yaml-cpp)
find_package(FLTK REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OOQP REQUIRED)
# 定义项目的包含目录路径，这些路径中的头文件可以在项目中直接包含。
set(OOQP_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/third_party/ooqp/include/ooqp)

set(INCLUDE_PATH 
    ${CMAKE_SOURCE_DIR}/include 
    ${Python3_INCLUDE_DIRS} 
    ${CMAKE_SOURCE_DIR}/third_party/matplotlib-cpp 
    ${CMAKE_SOURCE_DIR}/third_party
    ${CMAKE_SOURCE_DIR}/third_party/yaml-cpp/include
    ${EIGEN3_INCLUDE_DIR}
    ${PYBIND11_INCLUDE_DIRS}
    ${FLTK_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${OOQP_INCLUDE_DIRS}
)

# file(GLOB_RECURSE SRC_FILES ${CMAKE_SOURCE_DIR}/src/*.cpp)
# aux_source_directory(${CMAKE_SOURCE_DIR}/src/common COMMON_SRC_FILES)
aux_source_directory(${CMAKE_SOURCE_DIR}/src/eudm_planner EUDM_SRC_FILES)
aux_source_directory(${CMAKE_SOURCE_DIR}/src/park_data_reader READER_SRC_FILES)
aux_source_directory(${CMAKE_SOURCE_DIR}/src/semantic_map_manager SEMANTIC_SRC_FILES)
aux_source_directory(${CMAKE_SOURCE_DIR}/src/vehicle_model VEHICLE_SRC_FILES)
aux_source_directory(${CMAKE_SOURCE_DIR}/src/visualizer VIS_SRC_FILES)
file(GLOB_RECURSE VEHICLE_SRC_FILES ${CMAKE_SOURCE_DIR}/src/vehicle_model/*.cc)
file(GLOB_RECURSE COMMON_SRC_FILES ${CMAKE_SOURCE_DIR}/src/common/*.cc)


# 添加一个可执行目标到项目中，并指定构建这个目标需要的源文件。
add_executable(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/test/main2.cpp 
                                ${COMMON_SRC_FILES}
                                ${EUDM_SRC_FILES}
                                ${READER_SRC_FILES}
                                ${SEMANTIC_SRC_FILES}
                                ${VEHICLE_SRC_FILES}
                                ${VIS_SRC_FILES}
)

# 为可执行目标设置私有包含目录，这意味着这些目录只对这个目标可见。
target_include_directories(${PROJECT_NAME} PRIVATE ${INCLUDE_PATH})

# 链接Python库和NumPy库到可执行目标，确保目标可以使用这些外部库的功能。
target_link_libraries(${PROJECT_NAME}
                      ${Python3_LIBRARIES} 
                      Python3::Python 
                      Python3::NumPy 
                      pybind11::module 
                      nlohmann_json::nlohmann_json
                      yaml-cpp
                      ${FLTK_LIBRARIES}
                      Threads::Threads
                      ${OpenCV_LIBS}
                      ${OOQP_LIBRARIES}
)


