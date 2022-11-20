cmake_minimum_required(VERSION 3.0)
project(rotation_update)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(rotation_update main.cpp)