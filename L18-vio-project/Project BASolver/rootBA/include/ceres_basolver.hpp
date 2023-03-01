#pragma once

#include <iostream>
#include <random>
#include "frame.hpp"
#include "ceres/ceres.h"
#include <ceres/rotation.h>
#include <Eigen/Core>
#include <Eigen/Geometry>


//using namespace ceres;
using namespace Eigen;
using namespace std;

struct ReprojectionError3D
{
    ReprojectionError3D(double observed_u, double observed_v)
        :observed_u(observed_u), observed_v(observed_v)
        {}

    template <typename T>
    bool operator()(const T* const camera_R, const T* const camera_T, const T* point, T* residuals) const
    {
        T p[3];
        T p_min_t[3];
        T camera_R_inv[4];
        //calculate residual ceres
         p_min_t[0] = point[0] - camera_T[0];
         p_min_t[1] = point[1] - camera_T[1];
         p_min_t[2] = point[2] - camera_T[2];
         //Inverse of roatation
        camera_R_inv[0] =  camera_R[0];
        camera_R_inv[1] = - camera_R[1];
        camera_R_inv[2] = - camera_R[2];
        camera_R_inv[3] = - camera_R[3];
        ceres::QuaternionRotatePoint(camera_R_inv, p_min_t, p);
        //p[0] += camera_T[0]; p[1] += camera_T[1]; p[2] += camera_T[2];
        T xp = p[0] / p[2];
        T yp = p[1] / p[2];
        residuals[0] = xp - T(observed_u);
        residuals[1] = yp - T(observed_v);

        return true;
    }

    static ceres::CostFunction* Create(const double observed_x,
                                       const double observed_y)
    {
      return (new ceres::AutoDiffCostFunction<
              ReprojectionError3D, 2, 4, 3, 3>(
                new ReprojectionError3D(observed_x,observed_y)));
    }

    double observed_u;
    double observed_v;
};


class ceresBAProblem{

    public:
    ceresBAProblem(std::vector<Frame>& cameras, std::vector<Eigen::Matrix<Scalar, 3, 1>>& points):problem()
    {
        frame_num_ = cameras.size();
        lm_num_ = points.size();

        lm_pose_ = new double*[lm_num_];
        for (int i = 0; i < lm_num_; i++)
        {
            lm_pose_[i]= new double[3];
        }

        c_rotation_ = new double*[frame_num_];
        c_translation_ = new double*[frame_num_];

        for (int i = 0; i < frame_num_; i++)
        {
            c_rotation_[i] = new double[4];
            c_translation_[i] = new double[3];
        }


        local_parameterization = new ceres::QuaternionParameterization();
        for (int i = 0; i < frame_num_; i++)
        {
            auto&& cam_q = cameras[i].qwc_noisy;
            auto& cam_t = cameras[i].twc_noisy;
            c_translation_[i][0] = static_cast<double>(cam_t.x());
            c_translation_[i][1] = static_cast<double>(cam_t.y());
            c_translation_[i][2] = static_cast<double>(cam_t.z());
            c_rotation_[i][0] = static_cast<double>(cam_q.w());
            c_rotation_[i][1] = static_cast<double>(cam_q.x());
            c_rotation_[i][2] = static_cast<double>(cam_q.y());
            c_rotation_[i][3] = static_cast<double>(cam_q.z());
            problem.AddParameterBlock(c_rotation_[i], 4, local_parameterization);
            problem.AddParameterBlock(c_translation_[i], 3);
            if(0 == i)
                {
                    problem.SetParameterBlockConstant(c_rotation_[i]);
                    problem.SetParameterBlockConstant(c_translation_[i]);
                }
        }

        for (int i = 0; i < lm_num_; i++)
        {
            for (int j = 0; j < frame_num_; j++)
            {
                lm_pose_[i][0] = static_cast<double>(points[i].x());
                lm_pose_[i][1] = static_cast<double>(points[i].y());
                lm_pose_[i][2] = static_cast<double>(points[i].z());
                cost_function = ReprojectionError3D::Create(
                                                    static_cast<double>(cameras[j].featurePerId[i].x()),
                                                    static_cast<double>(cameras[j].featurePerId[i].y()));

                problem.AddResidualBlock(cost_function, NULL, c_rotation_[j], c_translation_[j],
                                        lm_pose_[i]);
            }

        }

    }

        ~ceresBAProblem()
    {
        for (int i = 0; i < frame_num_; i++)
        {
            delete[] c_rotation_[i];
            delete[] c_translation_[i];
        }
        delete [] c_rotation_;
        delete [] c_translation_;
        for (int i = 0; i < lm_num_; i++)
        {
            delete[] lm_pose_[i];
        }
        delete[] lm_pose_;
    }

    void generateResult(std::vector<Frame>& cameras, std::vector<Eigen::Matrix<Scalar, 3, 1>>& points)
    {
        for (int i = 0; i < frame_num_; i++)
        {
            Eigen::Quaternion<Scalar> q;
            q.w() = static_cast<Scalar>(c_rotation_[i][0]);
            q.x() = static_cast<Scalar>(c_rotation_[i][1]);
            q.y() = static_cast<Scalar>(c_rotation_[i][2]);
            q.z() = static_cast<Scalar>(c_rotation_[i][3]);
            Eigen::Matrix<Scalar, 3, 1> t = Eigen::Map<VectorXd>(c_translation_[i], 3).cast<Scalar>();
            cameras.emplace_back(q,t);
        }

        for (int i = 0; i < lm_num_; i++)
        {
            Eigen::Matrix<Scalar, 3, 1> t = Eigen::Map<VectorXd>(lm_pose_[i], 3).cast<Scalar>();
            points.push_back(t);
        }
    }

    void set_options()
    {
        options.max_num_iterations = 25;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.minimizer_progress_to_stdout = true;
        options.num_threads = 4;
    }

    void solve()
    {
        ceres::Solve(options, &problem, &summary);
    };

    void print_summary(int verbose_lvl) {
        if (verbose_lvl == 0) {
            // nothing
        } else if (verbose_lvl == 1) {
            // brief report
            std::cout << summary.BriefReport() << "\n";
        } else {
            // >= 2: full report
            std::cout << summary.FullReport() << "\n";
        }
    }

    private:
        int frame_num_;
        int lm_num_;
        double** c_rotation_;
        double** c_translation_;
        double** lm_pose_;
        ceres::LocalParameterization* local_parameterization;
        ceres::Problem problem;
        ceres::CostFunction* cost_function;
        ceres::Solver::Options options; // 配置
        ceres::Solver::Summary summary;

};
