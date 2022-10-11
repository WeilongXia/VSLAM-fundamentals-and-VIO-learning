#include "hello.h"
// #include <iostream>
#include <glog/logging.h>

bool sayHello() 
{
    // std::cout<<"Hello SLAM"<<std::endl;

    LOG(INFO) << "Hello SLAM";

    return true;
}
