#include <iostream>

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>

Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d &w)
{
    Eigen::Matrix3d mat;
    mat << 0, -w.z(), w.y(), w.z(), 0, -w.x(), -w.y(), w.x(), 0;
    return mat;
}

int main()
{
    Eigen::Vector3d w(0.01, 0.02, 0.03);

    Eigen::Vector4d q_random = Eigen::Vector4d::Random();
    q_random.normalize();
    std::cout << "q_random: " << q_random.transpose() << std::endl;
    std::cout << "q_random norm: " << q_random.norm() << std::endl;
    Eigen::Quaterniond q_init(q_random[0], q_random[1], q_random[2], q_random[3]);

    Eigen::Matrix3d R_init = q_init.toRotationMatrix();

    Eigen::Quaterniond delta_q(1, 0.5 * w.x(), 0.5 * w.y(), 0.5 * w.z());
    Eigen::Quaterniond q_update = q_init * delta_q;
    q_update.normalize();
    Eigen::Matrix3d R_update = R_init * skew_symmetric(w).exp();
    Eigen::Quaterniond q_verify(R_update);

    std::cout << "q_update: " << q_update.w() << " " << q_update.x() << " " << q_update.y() << " " << q_update.z()
              << std::endl;
    std::cout << "q_verify: " << q_verify.w() << " " << q_verify.x() << " " << q_verify.y() << " " << q_verify.z()
              << std::endl;

    return 0;
}