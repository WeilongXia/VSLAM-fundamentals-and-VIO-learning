//
// Created by gaoxiang on 19-5-4.
//
#include "myslam/visual_odometry.h"
#include "myslam/config.h"
#include <chrono>
#include <iomanip>
#include <iterator>
#include <string>

namespace myslam
{

VisualOdometry::VisualOdometry(std::string &config_path) : config_file_path_(config_path)
{
}

bool VisualOdometry::Init()
{
    // read from config file
    if (Config::SetParameterFile(config_file_path_) == false)
    {
        std::cout << "set parameter failed" << std::endl;
        return false;
    }

    std::cout << "num_features: " << Config::Get<int>("num_features") << std::endl;
    dataset_ = Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir")));
    CHECK_EQ(dataset_->Init(), true);

    // create components and links
    frontend_ = Frontend::Ptr(new Frontend);
    backend_ = Backend::Ptr(new Backend);
    map_ = Map::Ptr(new Map);
    viewer_ = Viewer::Ptr(new Viewer);

    frontend_->SetBackend(backend_);
    frontend_->SetMap(map_);
    frontend_->SetViewer(viewer_);
    frontend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

    backend_->SetMap(map_);
    backend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

    viewer_->SetMap(map_);

    return true;
}

void VisualOdometry::Run()
{
    while (1)
    {
        LOG(INFO) << "VO is running";
        if (Step() == false)
        {
            break;
        }
    }

    backend_->Stop();
    viewer_->Close();

    LOG(INFO) << "VO exit";
}

bool VisualOdometry::Step()
{
    Frame::Ptr new_frame = dataset_->NextFrame();
    if (new_frame == nullptr)
        return false;

    auto t1 = std::chrono::steady_clock::now();
    bool success = frontend_->AddFrame(new_frame);
    auto t2 = std::chrono::steady_clock::now();
    auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    if (frontend_->Twc().lpNorm<1>() != 0)
    {
        // std::cout << "Twc: " << frontend_->Twc() << std::endl;
        mlRelativeFramePoses_.push_back(frontend_->Twc());
    }
    // std::cout << "size: " << mlRelativeFramePoses_.size() << std::endl;
    LOG(INFO) << "VO cost time: " << time_used.count() << " seconds.";
    return success;
}

void VisualOdometry::SaveTrajectoryKITTI(const std::string &filename)
{
    std::cout << std::endl << "Saving camera trajectory to " << filename << "... " << std::endl;

    std::ofstream f;
    f.open(filename.c_str());
    f << std::fixed;

    for (std::list<Mat44>::iterator lit = mlRelativeFramePoses_.begin(), lend = mlRelativeFramePoses_.end();
         lit != lend; lit++)
    {
        Mat44 Twc = *lit;
        f << std::setprecision(9) << Twc(0, 0) << " " << Twc(0, 1) << " " << Twc(0, 2) << " " << Twc(0, 3) << " "
          << Twc(1, 0) << " " << Twc(1, 1) << " " << Twc(1, 2) << " " << Twc(1, 3) << " " << Twc(2, 0) << " "
          << Twc(2, 1) << " " << Twc(2, 2) << " " << Twc(2, 3) << std::endl;
    }
    f.close();
    std::cout << std::endl << "trajectory saved!" << std::endl;
}

} // namespace myslam
