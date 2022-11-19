//
// Created by gaoxiang on 19-5-4.
//

#include "myslam/visual_odometry.h"
#include <gflags/gflags.h>

DEFINE_string(config_file, "../config/default.yaml", "config file path");

int main(int argc, char **argv)
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    myslam::VisualOdometry::Ptr vo(new myslam::VisualOdometry(FLAGS_config_file));
    std::cout << "init" << std::endl;
    assert(vo->Init() == true);
    std::cout << "init" << std::endl;
    vo->Run();

    vo->SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    return 0;
}
