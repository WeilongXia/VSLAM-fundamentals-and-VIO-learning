cmake_minimum_required(VERSION 3.10)
project(solve_equation)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIRS})
message(STATUS "EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(solver solve_equation.cpp)

