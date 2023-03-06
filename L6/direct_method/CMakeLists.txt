cmake_minimum_required(VERSION 3.0)
project(direct_method)

set(CMAKE_BUILD_TYPE "Release")
add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS "-std=c++11 ${SSE_FLAGS} -g -O3 -march=native")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

find_package(OpenCV 4 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)

add_executable(direct_method direct_method.cpp)
target_link_libraries(direct_method ${OpenCV_LIBS} ${Pangolin_LIBRARIES} fmt)

add_executable(calc_disparity calc_disparity.cpp)
target_link_libraries(calc_disparity ${OpenCV_LIBS} ${Pangolin_LIBRARIES})