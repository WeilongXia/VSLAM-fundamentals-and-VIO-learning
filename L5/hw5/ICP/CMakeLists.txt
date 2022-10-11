cmake_minimum_required(VERSION 3.10)
project(compare)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Eigen3 REQUIRED)
set(Sophus_LIBRARIES libSophus.so)

include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${Sophus_INCLUDE_DIRS})

add_executable(ICP ICP.cpp)
target_link_libraries(ICP ${Pangolin_LIBRARIES} ${Sophus_LIBRARIES})