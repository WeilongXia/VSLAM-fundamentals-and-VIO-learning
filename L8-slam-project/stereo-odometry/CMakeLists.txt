cmake_minimum_required(VERSION 2.8)

project(stereo-odometry)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(Stereo_SLAM)
target_link_libraries(Stereo_SLAM ${OpenCV_LIBS})

add_executable(stereo_odometry stereo_odometry.cpp)
target_link_libraries(stereo_odometry Stereo_SLAM)



