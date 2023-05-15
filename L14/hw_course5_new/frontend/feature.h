//
// Created by gaoxiang19 on 19-1-7.
//

#ifndef SLAM_COURSE_FEATURE_H
#define SLAM_COURSE_FEATURE_H

#include "backend/eigen_types.h"
#include <memory>

namespace myslam
{
namespace frontend
{

struct Frame;
struct MapPoint;

struct Feature
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    explicit Feature(const Vec2f &pixel, std::shared_ptr<Frame> ref)
    {
        ref_frame = ref;
    }

    Vec2f pixel_pos = Vec2f::Zero();
    std::weak_ptr<Frame> ref_frame;
    bool outlier = false;
    std::weak_ptr<MapPoint> map_point;
};

} // namespace frontend
} // namespace myslam

#endif // SLAM_COURSE_FEATURE_H
