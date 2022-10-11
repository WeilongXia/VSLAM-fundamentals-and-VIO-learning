cmake_minimum_required(VERSION 3.0)
project(learn_cmake)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(Hello REQUIRED)
include_directories(${PROJECT_SOURCE_DIR}/include ${HELLO_INCLUDE_DIR})

# generate shared library libhello.so
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_library(hello SHARED src/hello.cc)

# generate executable file sayHello
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
add_executable(sayHello src/useHello.cc)
target_link_libraries(sayHello ${HELLO_LIBRARY})

message(STATUS ${HELLO_LIBRARY})
message(STATUS ${HELLO_INCLUDE_DIR})

# install libhello.so to /usr/local/lib 
install(TARGETS hello
        LIBRARY DESTINATION lib
)
# install hello.h to /usr/local/include
install(FILES include/hello.h 
        DESTINATION include
)