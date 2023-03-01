#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <map>
#include <memory>
#include <list>
#include <vector>
#include <unordered_map>

//template <class Scalar> //声明一个模板，虚拟类型名为T。注意：这里没有分号。
using Scalar = float;
struct Frame {
    Frame(Eigen::Matrix<Scalar, 3, 3> R, Eigen::Matrix<Scalar, 3, 1> t) : Rwc(R), qwc(R), twc(t), qcw(R.transpose()), tcw(-qcw.toRotationMatrix()*t) {};
    Frame(Eigen::Quaternion<Scalar> R, Eigen::Matrix<Scalar, 3, 1> t) : qwc(R), twc(t),qcw(R.inverse()),tcw(-qcw.toRotationMatrix()*t) {};

    Eigen::Matrix<Scalar, 3, 3> Rwc;
    Eigen::Quaternion<Scalar> qwc;
    Eigen::Matrix<Scalar, 3, 1> twc;
    Eigen::Matrix<Scalar, 3, 3> Rcw;
    Eigen::Quaternion<Scalar> qcw;
    Eigen::Matrix<Scalar, 3, 1> tcw;

    Eigen::Quaternion<Scalar>  qwc_noisy;
    Eigen::Matrix<Scalar, 3, 1> twc_noisy;
    Eigen::Quaternion<Scalar>  qcw_noisy;
    Eigen::Matrix<Scalar, 3, 1> tcw_noisy;
    std::unordered_map<int,  Eigen::Matrix<Scalar, 3, 1>> featurePerId; // 该帧观测到的特征以及特征id

    void setNoise ( Eigen::Matrix<Scalar, 3, 1> twc_noise, Eigen::Quaternion<Scalar> qwc_noise)
    {
        qwc_noisy = qwc * qwc_noise;
        twc_noisy = twc + twc_noise;
        qcw_noisy = qwc_noisy.inverse();
        tcw_noisy = - qcw_noisy.toRotationMatrix() * twc_noisy;
    }
};