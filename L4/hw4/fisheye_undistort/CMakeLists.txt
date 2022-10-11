cmake_minimum_required(VERSION 3.0)
project(fisheye_undistort)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(fisheye_undistort fisheye.cpp)
target_link_libraries(fisheye_undistort ${OpenCV_LIBS})