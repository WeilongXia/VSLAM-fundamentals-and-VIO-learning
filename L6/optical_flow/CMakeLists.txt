cmake_minimum_required(VERSION 3.0)
project(optical_flow)

set(CMAKE_BUILD_TYPE "Release")
add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS "-std=c++11 ${SSE_FLAGS} -g -O3 -march=native")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

add_executable(optical_flow optical_flow.cpp)
target_link_libraries(optical_flow ${OpenCV_LIBS})