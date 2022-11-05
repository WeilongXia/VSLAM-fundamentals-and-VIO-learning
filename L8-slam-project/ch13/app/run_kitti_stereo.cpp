//
// Created by gaoxiang on 19-5-4.
//

#include "myslam/visual_odometry.h"
#include <gflags/gflags.h>

DEFINE_string(config_file, "../config/default.yaml", "config file path");
// std::string config_file = "../config/default.yaml";

int main(int argc, char **argv)
{
    // std::cout << "init" << std::endl;
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    myslam::VisualOdometry::Ptr vo(new myslam::VisualOdometry(FLAGS_config_file));
    // myslam::VisualOdometry::Ptr vo(new myslam::VisualOdometry(config_file));
    std::cout << "init" << std::endl;
    assert(vo->Init() == true);
    std::cout << "init" << std::endl;
    vo->Run();

    return 0;
}
