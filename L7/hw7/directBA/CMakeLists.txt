cmake_minimum_required(VERSION 2.8)

project(direct_BA)
set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "-O3 -std=c++17")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

Find_Package(G2O REQUIRED)
Find_Package(Eigen3 REQUIRED)
Find_Package(Sophus REQUIRED)
Find_Package(CSparse REQUIRED)
Find_Package(OpenCV REQUIRED)
Find_Package(Pangolin REQUIRED)

SET(G2O_LIBS g2o_csparse_extension g2o_stuff g2o_core g2o_types_sba cxsparse fmt)

include_directories(${PROJECT_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS})

add_executable(direct_BA directBA.cpp)

target_link_libraries(direct_BA ${G2O_LIBS} ${OpenCV_LIBS} ${Pangolin_LIBRARIES})
