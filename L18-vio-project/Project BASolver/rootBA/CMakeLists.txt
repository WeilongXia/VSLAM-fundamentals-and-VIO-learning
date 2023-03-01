# cmake工程最低版本为3.10
cmake_minimum_required( VERSION 3.1 )

# cmake工程名
project( rootBA )

# 设置编译类型为发布
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/" ${CMAKE_MODULE_PATH})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# 添加c++ 17标准的支持
set( CMAKE_CXX_FLAGS "-std=c++17 -O3" )

# 寻找 TBB 库
find_package( TBB REQUIRED )

# 寻找 Eigen 库并添加其路径
find_package( Eigen3 REQUIRED )

find_package( Ceres REQUIRED )

#include_directories( "/usr/include/eigen3" )

# 添加自定义路径
include_directories( ${PROJECT_SOURCE_DIR} )
include_directories( ${PROJECT_SOURCE_DIR}/include )
include_directories( ${PROJECT_SOURCE_DIR}/test )
include_directories( ${CERES_INCLUDE_DIRS} )
include_directories(${EIGEN3_INCLUDE_DIR})

# 添加可执行文件 test_rootba
add_executable( test_rootba
    src/test_rootba.cpp
)
# 链接库到可执行文件
target_link_libraries( test_rootba
    TBB::tbb
)

# 添加可执行文件 test_rootba_marg
add_executable( benchmark_rootba
    src/benchmark_rootba.cpp
)
# 链接库到可执行文件
target_link_libraries( benchmark_rootba
TBB::tbb
${CERES_LIBRARIES}
)