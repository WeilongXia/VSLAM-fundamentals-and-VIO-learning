#include "ceres_basolver.hpp"
#include "frame.hpp"
#include "problem_vo.hpp"
#include <iostream>
#include <random>

using CameraPtr = std::shared_ptr<rootBA::VertexCameraPose<Scalar>>;
using LandmarkPtr = std::shared_ptr<rootBA::VertexLandmarkPosi<Scalar>>;
/*
 * 产生世界坐标系下的虚拟数据: 相机姿态, 特征点, 以及每帧观测
 */
void GetSimDataInWordFrame(std::vector<Frame> &cameraPoses, std::vector<rootBA::Vector3<Scalar>> &points,
                           int featureNums, int poseNums)
{
    double radius = 8;
    for (int n = 0; n < poseNums; ++n)
    {
        double theta = n * 2 * M_PI / (poseNums * 16); // 1/16 圆弧
        // 绕 z 轴 旋转
        rootBA::Matrix3<Scalar> R;
        R = Eigen::AngleAxis<Scalar>(theta, rootBA::Vector3<Scalar>::UnitZ());
        rootBA::Vector3<Scalar> t =
            rootBA::Vector3<Scalar>(radius * cos(theta) - radius, radius * sin(theta), 1 * sin(2 * theta));
        cameraPoses.push_back(Frame(R, t));
    }

    // 随机数生成三维特征点
    std::default_random_engine generator;
    std::normal_distribution<double> noise_pdf(0., 1. / 1000.); // 2pixel / focal
    for (int j = 0; j < featureNums; ++j)
    {
        std::uniform_real_distribution<double> xy_rand(-4, 4.0);
        std::uniform_real_distribution<double> z_rand(4., 8.);

        rootBA::Vector3<Scalar> Pw(xy_rand(generator), xy_rand(generator), z_rand(generator));
        points.push_back(Pw);

        // 在每一帧上的观测量
        for (int i = 0; i < poseNums; ++i)
        {
            rootBA::Vector3<Scalar> Pc = cameraPoses[i].Rwc.transpose() * (Pw - cameraPoses[i].twc);
            if (Pc.z() < 1e-10)
            {
                continue;
            }
            Pc = Pc / Pc.z(); // 归一化图像平面
            Pc[0] += noise_pdf(generator);
            Pc[1] += noise_pdf(generator);
            cameraPoses[i].featurePerId.insert(std::make_pair(j, Pc));
        }
    }
}

void AddNoiseinCamera(std::vector<Frame> &cameras)
{
    std::normal_distribution<double> camera_rotation_noise(0., 0.1);
    std::normal_distribution<double> camera_position_noise(0., 0.2);
    std::default_random_engine generator;

    for (size_t i = 0; i < cameras.size(); ++i)
    {
        // 给相机位置和姿态初值增加噪声
        rootBA::Matrix3<Scalar> noise_R, noise_X, noise_Y, noise_Z;
        noise_X = Eigen::AngleAxis<Scalar>(camera_rotation_noise(generator), rootBA::Vector3<Scalar>::UnitX());
        noise_Y = Eigen::AngleAxis<Scalar>(camera_rotation_noise(generator), rootBA::Vector3<Scalar>::UnitY());
        noise_Z = Eigen::AngleAxis<Scalar>(camera_rotation_noise(generator), rootBA::Vector3<Scalar>::UnitZ());
        noise_R = noise_X * noise_Y * noise_Z;
        rootBA::Vector3<Scalar> noise_t(camera_position_noise(generator), camera_position_noise(generator),
                                        camera_position_noise(generator));
        rootBA::Quaternion<Scalar> noise_q_wc(noise_R);
        if (i < 2)
        {
            noise_t.setZero();
            noise_q_wc.setIdentity();
        }
        cameras[i].setNoise(noise_t, noise_q_wc);
    }
}

void GetSimNoisyLandmark(std::vector<rootBA::Vector3<Scalar>> &points, std::vector<rootBA::Vector3<Scalar>> &noised_pts)
{
    for (size_t i = 0; i < points.size(); ++i)
    {
        // 为初值添加随机噪声
        std::default_random_engine generator;
        std::normal_distribution<double> landmark_position_noise(0., 0.5);
        rootBA::Vector3<Scalar> noise(landmark_position_noise(generator), landmark_position_noise(generator),
                                      landmark_position_noise(generator));
        // TODO check double/flaot
        rootBA::Vector3<Scalar> noised_pt = points[i] + noise;
        noised_pts.push_back(noised_pt);
    }
}

/* 程序主函数入口 */
int main()
{
    std::cout << "Test rootBA solver on Mono BA problem." << std::endl;

    // 第一步：准备测试数据
    std::cout << "\nStep 1: Prepare dataset." << std::endl;
    std::vector<Frame> cameras, cameras_esti;
    std::vector<rootBA::Vector3<Scalar>> points, noisy_points, points_esti;
    GetSimDataInWordFrame(cameras, points, 300, 10);
    GetSimNoisyLandmark(points, noisy_points);
    AddNoiseinCamera(cameras);

    ceresBAProblem ceres_problem(cameras, noisy_points);
    ceres_problem.set_options();
    rootBA::TicToc timer;
    ceres_problem.solve();
    auto ceres_time = timer.toc();
    std::cout << "<ceres> cost time " << ceres_time << std::endl;
    ceres_problem.generateResult(cameras_esti, points_esti);

    // 第二步：构造 BA 问题求解器，并添加相机与 IMU 外参节点
    std::cout << "\nStep 2: Construct problem solver." << std::endl;
    rootBA::ProblemVO<Scalar> problem;
    problem.Reset();
    rootBA::Quaternion<Scalar> q_bc;
    q_bc.setIdentity();
    rootBA::Vector3<Scalar> t_bc = rootBA::Vector3<Scalar>::Zero();
    std::shared_ptr<rootBA::VertexExPose<Scalar>> exPose(new rootBA::VertexExPose<Scalar>(q_bc, t_bc));
    problem.AddExPose(exPose);
    exPose->SetFixed();
    std::cout << "problem has 1 ex pose." << std::endl;

    // 第三步：构造相机 pose 节点，并添加到 problem 中
    std::cout << "\nStep 3: Add camera pose vertices." << std::endl;
    std::default_random_engine generator;
    std::normal_distribution<double> camera_rotation_noise(0., 0.1);
    std::normal_distribution<double> camera_position_noise(0., 0.2);
    std::vector<CameraPtr> cameraVertices;
    for (size_t i = 0; i < cameras.size(); ++i)
    {
        CameraPtr cameraVertex(new rootBA::VertexCameraPose<Scalar>(cameras[i].qwc_noisy, cameras[i].twc_noisy));
        cameraVertices.emplace_back(cameraVertex);
        problem.AddCamera(cameraVertex);
    }
    std::cout << "problem has " << problem.GetCamerasNum() << " cameras." << std::endl;

    // 第四步：构造特征点 position 节点，并添加到 problem 中
    std::cout << "\nStep 4: Add landmark position vertices." << std::endl;
    std::list<LandmarkPtr> landmarkVertices;
    size_t maxObserveNum = cameras.size() / 2;
    for (size_t i = 0; i < points.size(); ++i)
    {
        LandmarkPtr landmarkVertex(new rootBA::VertexLandmarkPosi<Scalar>(noisy_points[i]));
        // 构造观测
        std::unordered_map<size_t, std::shared_ptr<rootBA::CameraObserve<Scalar>>> observes;
        --maxObserveNum;
        for (size_t j = 0; j < cameras.size(); ++j)
        {
            std::shared_ptr<rootBA::HuberKernel<Scalar>> kernel(new rootBA::HuberKernel<Scalar>(1.0));
            std::shared_ptr<rootBA::CameraObserve<Scalar>> observe(new rootBA::CameraObserve<Scalar>(
                cameraVertices[j], cameras[j].featurePerId.find(i)->second.head<2>(), kernel));
            observes.insert(std::make_pair(j, observe));
        }
        if (maxObserveNum == 0)
        {
            maxObserveNum = cameras.size() / 2;
        }

        landmarkVertices.emplace_back(landmarkVertex);
        problem.AddLandmark(landmarkVertex, observes);
    }
    std::cout << "problem has " << problem.GetLandmarksNum() << " landmarks." << std::endl;

    // 第五步：打印出优化求解的初值
    std::cout << "\nStep 5: Show the initial value of parameters." << std::endl;
    std::cout << "=================== Initial parameters -> Landmark Position ===================" << std::endl;
    size_t i = 0;
    for (auto landmark : landmarkVertices)
    {
        std::cout << "  id " << i << " : gt [" << points[i].transpose() << "],\top [" << landmark->Get_p_w().transpose()
                  << "]" << std::endl;
        ++i;
        if (i > 10)
        {
            break;
        }
    }
    std::cout << "=================== Initial parameters -> Camera Rotation ===================" << std::endl;
    for (size_t i = 0; i < cameraVertices.size(); ++i)
    {
        std::cout << "  id " << i << " : gt [" << cameras[i].qwc.w() << ", " << cameras[i].qwc.x() << ", "
                  << cameras[i].qwc.y() << ", " << cameras[i].qwc.z() << "],\top [" << cameraVertices[i]->Get_q_wb().w()
                  << ", " << cameraVertices[i]->Get_q_wb().x() << ", " << cameraVertices[i]->Get_q_wb().y() << ", "
                  << cameraVertices[i]->Get_q_wb().z() << "]" << std::endl;
    }
    std::cout << "=================== Initial parameters -> Camera Position ===================" << std::endl;
    for (size_t i = 0; i < cameraVertices.size(); ++i)
    {
        std::cout << "  id " << i << " : gt [" << cameras[i].twc.transpose() << "],\top ["
                  << cameraVertices[i]->Get_t_wb().transpose() << "]" << std::endl;
    }

    // 第六步：设置求解器的相关参数，开始求解
    std::cout << "\nStep 6: Start solve problem." << std::endl;
    problem.SetThreshold(1e-6, 1, 1e-6, 1); // 设置终止迭代判断阈值
    // problem.SetDampParameter(3.0, 9.0);     // 设置阻尼因子调整策略 1 的 Lup 和 Ldown 参数
    cameraVertices[0]->SetFixed(true); // 因为是 VO 问题，固定前两帧相机位姿，以固定尺度
    cameraVertices[1]->SetFixed(true);
    exPose->SetFixed(true);
    problem.SetDampPolicy(rootBA::ProblemVO<Scalar>::DampPolicy::Auto);
    problem.SetLinearSolver(rootBA::ProblemVO<Scalar>::LinearSolver::PCG);
    problem.Solve(100); // 求解问题，设置最大迭代步数
    // problem.Test();

    // 第七步：从 CameraClass 对象和 LandmarkClass 对象中提取出求解结果，对比真值
    std::cout << "\nStep 7: Compare optimization result with ground truth." << std::endl;
    std::cout << "=================== Solve result -> Landmark Position ===================" << std::endl;
    i = 0;
    for (auto landmark : landmarkVertices)
    {
        std::cout << "  id " << i << " : gt [" << points[i].transpose() << "],\top [" << landmark->Get_p_w().transpose()
                  << "]" << std::endl;
        ++i;
        if (i > 10)
        {
            break;
        }
    }
    std::cout << "=================== Solve result -> Camera Rotation ===================" << std::endl;
    for (size_t i = 0; i < cameraVertices.size(); ++i)
    {
        std::cout << "  id " << i << " : gt [" << cameras[i].qwc.w() << ", " << cameras[i].qwc.x() << ", "
                  << cameras[i].qwc.y() << ", " << cameras[i].qwc.z() << "],\top [" << cameraVertices[i]->Get_q_wb().w()
                  << ", " << cameraVertices[i]->Get_q_wb().x() << ", " << cameraVertices[i]->Get_q_wb().y() << ", "
                  << cameraVertices[i]->Get_q_wb().z() << "]" << std::endl;
    }
    std::cout << "=================== Solve result -> Camera Position ===================" << std::endl;
    for (size_t i = 0; i < cameraVertices.size(); ++i)
    {
        std::cout << "  id " << i << " : gt [" << cameras[i].twc.transpose() << "],\top ["
                  << cameraVertices[i]->Get_t_wb().transpose() << "]" << std::endl;
    }

    std::cout << "\nStep 8: Compute average residual." << std::endl;
    Scalar residual = 0.0;
    Scalar residual_ceres = 0.0;
    Scalar residual_init = 0.0;
    int cnt = 0;
    for (auto landmark : landmarkVertices)
    {
        rootBA::Vector3<Scalar> diff = points[cnt] - landmark->Get_p_w();
        rootBA::Vector3<Scalar> diff_ceres = points[cnt] - points_esti[cnt];
        rootBA::Vector3<Scalar> diff_init = points[cnt] - noisy_points[cnt];
        residual_ceres += rootBA::ComputeTranslationMagnitude(diff_ceres);
        residual += rootBA::ComputeTranslationMagnitude(diff);
        residual_init += rootBA::ComputeTranslationMagnitude(diff_init);
        ++cnt;
    }

    std::cout << "  Average landmark position residual for initialization is " << residual_init / Scalar(cnt) << " m"
              << std::endl;
    std::cout << "  Average landmark position residual for rootBA is \t" << residual / Scalar(cnt) << " m" << std::endl;
    std::cout << "  Average landmark position residual for CERES is \t" << residual_ceres / Scalar(cnt) << " m"
              << std::endl;

    residual = Scalar(0);
    residual_ceres = Scalar(0);
    residual_init = Scalar(0);
    for (size_t i = 0; i < cameraVertices.size(); ++i)
    {
        rootBA::Quaternion<Scalar> diff = cameras[i].qwc.inverse() * cameraVertices[i]->Get_q_wb();
        rootBA::Quaternion<Scalar> diff_ceres = cameras[i].qwc.inverse() * cameras_esti[i].qwc;
        rootBA::Quaternion<Scalar> diff_init = cameras[i].qwc.inverse() * cameras[i].qwc_noisy;
        residual += rootBA::ComputeRotationMagnitude(diff);
        residual_ceres += rootBA::ComputeRotationMagnitude(diff_ceres);
        residual_init += rootBA::ComputeRotationMagnitude(diff_init);
    }
    std::cout << "  Average camera atitude residual for RootBA is \t" << residual / Scalar(cameras.size()) << " rad, "
              << residual / Scalar(cameras.size()) * 57.29578049 << " deg" << std::endl;
    std::cout << "  Average camera atitude residual for Ceres is \t" << residual_ceres / Scalar(cameras.size())
              << " rad, " << residual_ceres / Scalar(cameras.size()) * 57.29578049 << " deg" << std::endl;
    std::cout << "  Average camera atitude residual at initialization is " << residual_init / Scalar(cameras.size())
              << " rad, " << residual_init / Scalar(cameras.size()) * 57.29578049 << " deg" << std::endl;

    residual = Scalar(0);
    residual_ceres = Scalar(0);
    residual_init = Scalar(0);
    for (size_t i = 0; i < cameraVertices.size(); ++i)
    {
        rootBA::Vector3<Scalar> diff = cameras[i].twc - cameraVertices[i]->Get_t_wb();
        rootBA::Vector3<Scalar> diff_ceres = cameras[i].twc - cameras_esti[i].twc;
        rootBA::Vector3<Scalar> diff_init = cameras[i].twc - cameras[i].twc_noisy;
        residual += rootBA::ComputeTranslationMagnitude(diff);
        residual_ceres += rootBA::ComputeTranslationMagnitude(diff_ceres);
        residual_init += rootBA::ComputeTranslationMagnitude(diff_init);
    }
    std::cout << "  Average camera position residual for RootBA is \t" << residual / Scalar(cameras.size()) << " m"
              << std::endl;
    std::cout << "  Average camera position residual for Ceres is \t" << residual_ceres / Scalar(cameras.size()) << " m"
              << std::endl;
    std::cout << "  Average camera position residual for initialization is " << residual_init / Scalar(cameras.size())
              << " m" << std::endl;

    return 0;
}