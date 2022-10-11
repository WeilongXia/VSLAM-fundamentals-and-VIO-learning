cmake_minimum_required(VERSION 3.10)
project(use_google_tk)

set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(PkgConfig REQUIRED)
find_package(gflags REQUIRED)
pkg_check_modules(glog REQUIRED libglog)
pkg_check_modules(gtest REQUIRED gtest)
include_directories(${PROJECT_SOURCE_DIR}/include ${glog_INCLUDE_DIRS})
# find_package(Hello REQUIRED)
# include_directories(${HELLO_INCLUDE_DIR})

# generate shared library libhello.so
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_library(hello SHARED src/hello.cc)
target_link_libraries(hello ${glog_LIBRARIES})

# generate executable file sayHello
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
add_executable(sayHello src/useHello.cc)
target_link_libraries(sayHello 
${PROJECT_SOURCE_DIR}/lib/libhello.so 
${glog_LIBRARIES} 
${gflags_LIBRARIES} 
${gtest_LIBRARIES}
)
# target_link_libraries(sayHello libhello.so)

# install libhello.so to /usr/local/lib 
# install(TARGETS hello LIBRARY DESTINATION /usr/local/lib)
# install libhello.so to /usr/local/include
# install(FILES include/hello.h DESTINATION /usr/local/include)