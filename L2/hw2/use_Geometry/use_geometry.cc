#include <iostream>
#include <iomanip>

#include <Eigen/Geometry>

int main(int argc, char** argv)
{
    // 从机器人坐标系到世界坐标系的转换
    Eigen::Quaterniond q_WR(0.55, 0.3, 0.2, 0.2);
    Eigen::Vector3d t_WR(0.1, 0.2, 0.3);
    q_WR.normalize();

    // 从Body系到机器人坐标系的转换
    Eigen::Quaterniond q_RB(0.99, 0., 0., 0.01);
    Eigen::Vector3d t_RB(0.05, 0., 0.5);
    q_RB.normalize();

    // 从激光雷达坐标系到Body坐标系的转换
    Eigen::Quaterniond q_BL(0.3, 0.5, 0, 20.1);
    Eigen::Vector3d t_BL(0.4, 0., 0.5);
    q_BL.normalize();

    // 从相机坐标系到Body坐标系的转换
    Eigen::Quaterniond q_BC(0.8, 0.2, 0.1, 0.1);
    Eigen::Vector3d t_BC(0.5, 0.1, 0.5);
    q_BC.normalize();

    Eigen::Vector3d P_C(0.3, 0.2, 1.2);

    Eigen::Vector3d P_L = q_BL.inverse() * (q_BC * P_C + t_BC) - q_BL.inverse() * t_BL;
    // Eigen::Vector3d P_L = q_BL.conjugate() * (q_BC * P_C + t_BC) - q_BL.conjugate() * t_BL;

    Eigen::Vector3d P_W = q_WR * (q_RB * (q_BC * P_C + t_BC) + t_RB) + t_WR;

    std::cout << "P_L: " << P_L.transpose() << std::endl;
    std::cout << "P_W: " << P_W.transpose() << std::endl;

    return 0;
}