cmake_minimum_required(VERSION 3.0)
project(GN_BA)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
set(Sophus_LIBRARIES libSophus.so)
# message("sophus_include_dirs: " ${Sophus_INCLUDE_DIRS})

add_executable(GN-BA GN-BA.cpp)
target_link_libraries(GN-BA ${Sophus_LIBRARIES})