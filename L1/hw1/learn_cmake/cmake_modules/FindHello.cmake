find_path(HELLO_INCLUDE_DIR hello.h /usr/local/include/)
find_library(HELLO_LIBRARY hello /usr/local/lib/)

if(HELLO_INCLUDE_DIR)
    message(STATUS "INCLUDE FIND!")
else()
    message(FATAL_ERROR "INCLUDE NOT FIND!")
endif(HELLO_INCLUDE_DIR)

if(HELLO_LIBRARY)
    message(STATUS "LIBRARY FIND!")
else()
    message(FATAL_ERROR "LIBRARY NOT FIND!")
endif(HELLO_LIBRARY)