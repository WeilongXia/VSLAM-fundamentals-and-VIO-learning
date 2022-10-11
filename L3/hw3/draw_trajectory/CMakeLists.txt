cmake_minimum_required(VERSION 3.10)
project(draw_trajectory)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${Sophus_INCLUDE_DIRS})

add_executable(draw_trajectory draw_trajectory.cpp)
target_link_libraries(draw_trajectory ${Pangolin_LIBRARIES} ${Sophus_LIBRARIES})