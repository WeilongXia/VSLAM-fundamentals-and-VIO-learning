//
// Created by gaoxiang on 19-5-4.
//
#include "myslam/visual_odometry.h"
#include "myslam/config.h"
#include <chrono>

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

    // std::cout << "1" << std::endl;
    // std::cout << "file: " << Config::Get<std::string>("dataset_dir") << std::endl;
    std::cout << "num_features: " << Config::Get<int>("num_features") << std::endl;
    dataset_ = Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir")));
    // std::cout << "file: " << Config::Get<std::string>("dataset_dir") << std::endl;
    // std::cout << "1" << std::endl;
    CHECK_EQ(dataset_->Init(), true);

    // std::cout << "1" << std::endl;

    // create components and links
    frontend_ = Frontend::Ptr(new Frontend);
    backend_ = Backend::Ptr(new Backend);
    map_ = Map::Ptr(new Map);
    viewer_ = Viewer::Ptr(new Viewer);

    std::cout << "1" << std::endl;

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
    LOG(INFO) << "VO cost time: " << time_used.count() << " seconds.";
    return success;
}

} // namespace myslam
