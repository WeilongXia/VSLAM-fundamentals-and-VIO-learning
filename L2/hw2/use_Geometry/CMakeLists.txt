cmake_minimum_required(VERSION 3.10)
project(use_Geometry)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 11)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(use_geometry use_geometry.cc)