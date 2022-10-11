//
// Created by xiang on 12/21/17.
//

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include "sophus/se3.h"

using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p3d_file = "../data/p3d.txt";
string p2d_file = "../data/p2d.txt";

int main(int argc, char **argv)
{

    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d K;
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    // load points in to p3d and p2d
    // START YOUR CODE HERE
    ifstream p3d_input_file(p3d_file);
    ifstream p2d_input_file(p2d_file);
    if (!p2d_input_file.is_open() || !p3d_input_file.is_open())
    {
        std::cout << "can not open files" << std::endl;
        return 0;
    }
    while (!p2d_input_file.eof())
    {
        double u, v;
        p2d_input_file >> u >> v;
        Eigen::Vector2d pos_pixel(u, v);
        p2d.emplace_back(pos_pixel);
    }
    while (!p3d_input_file.eof())
    {
        double x, y, z;
        p3d_input_file >> x >> y >> z;
        Eigen::Vector3d pos_world(x, y, z);
        p3d.emplace_back(pos_world);
    }
    // END YOUR CODE HERE
    assert(p3d.size() == p2d.size());

    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    cout << "points: " << nPoints << endl;

    Sophus::SE3 T_esti; // estimated pose

    for (int iter = 0; iter < iterations; iter++)
    {

        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();

        cost = 0;
        // compute cost
        for (int i = 0; i < nPoints; i++)
        {
            // compute cost for p3d[I] and p2d[I]
            // START YOUR CODE HERE
            Eigen::Vector3d pos_cam = T_esti * p3d[i];
            Eigen::Vector2d uv_esti(fx * pos_cam[0] / pos_cam[2] + cx, fy * pos_cam[1] / pos_cam[2] + cy);
            Eigen::Vector2d e = p2d[i] - uv_esti;

            cost += e.squaredNorm() * 0.5f;
            // std::cout << "cost: " << cost << std::endl;
            // END YOUR CODE HERE

            // compute jacobian
            Matrix<double, 2, 6> J;
            // START YOUR CODE HERE
            double inv_z = 1.0f / pos_cam[2];
            double inv_z2 = inv_z * inv_z;
            J << -fx * inv_z, 0, fx * pos_cam[0] * inv_z2, fx * pos_cam[0] * pos_cam[1] * inv_z2,
                -fx - fx * pow(pos_cam[0], 2) * inv_z2, fx * pos_cam[1] * inv_z, 0, -fy * inv_z,
                fy * pos_cam[1] * inv_z2, fy + fy * pow(pos_cam[1], 2) * inv_z2, -fy * pos_cam[0] * pos_cam[1] * inv_z2,
                -fy * pos_cam[0] * inv_z;

            // END YOUR CODE HERE

            H += J.transpose() * J;
            b += -J.transpose() * e;
        }

        // solve dx
        Vector6d dx;

        // START YOUR CODE HERE
        dx = H.householderQr().solve(b);
        // END YOUR CODE HERE

        if (isnan(dx[0]))
        {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost)
        {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // update your estimation
        // START YOUR CODE HERE
        T_esti = Sophus::SE3::exp(dx) * T_esti;
        // T_esti = T_esti * Sophus::SE3::exp(dx);
        // END YOUR CODE HERE

        lastCost = cost;

        cout << "iteration " << iter << " cost=" << cout.precision(12) << cost << endl;
    }

    cout << "estimated pose: \n" << T_esti.matrix() << endl;
    return 0;
}
