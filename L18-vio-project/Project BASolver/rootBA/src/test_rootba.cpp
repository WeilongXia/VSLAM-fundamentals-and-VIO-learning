#include <iostream>
#include <random>

#include <problem_vo.hpp>

using Scalar = float; // 方便切换浮点数精度

// /*
//  * Frame : 保存每帧的姿态和观测
//  */
// struct Frame {
//     Frame(rootBA::Matrix3<Scalar> R, rootBA::Vector3<Scalar> t) : Rwc(R), qwc(R), twc(t) {};
//     rootBA::Matrix3<Scalar> Rwc;
//     rootBA::Quaternion<Scalar> qwc;
//     rootBA::Vector3<Scalar> twc;
//     std::unordered_map<int, rootBA::Vector3<Scalar>> featurePerId; // 该帧观测到的特征以及特征id
// };

/*
 * 产生世界坐标系下的虚拟数据: 相机姿态, 特征点, 以及每帧观测
 */
void GetSimDataInWordFrame(std::vector<Frame> &cameraPoses, std::vector<rootBA::Vector3<Scalar>> &points)
{
    int featureNums = 300; // 特征数目，假设每帧都能观测到所有的特征
    int poseNums = 10;     // 相机数目

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

/* 程序主函数入口 */
int main()
{
    std::cout << "Test rootBA solver on Mono BA problem." << std::endl;

    // 第一步：准备测试数据
    std::cout << "\nStep 1: Prepare dataset." << std::endl;
    std::vector<Frame> cameras;
    std::vector<rootBA::Vector3<Scalar>> points;
    GetSimDataInWordFrame(cameras, points);

    // 第二步：构造 BA 问题求解器，并添加相机与 IMU 外参节点
    std::cout << "\nStep 2: Construct problem solver." << std::endl;
    rootBA::ProblemVO<Scalar> problem;
    problem.Reset();
    rootBA::Quaternion<Scalar> q_bc;
    q_bc.setIdentity();
    rootBA::Vector3<Scalar> t_bc = rootBA::Vector3<Scalar>::Zero();
    std::shared_ptr<rootBA::VertexExPose<Scalar>> exPose(new rootBA::VertexExPose<Scalar>(q_bc, t_bc));
    problem.AddExPose(exPose);
    std::cout << "problem has 1 ex pose." << std::endl;

    // 第三步：构造相机 pose 节点，并添加到 problem 中
    std::cout << "\nStep 3: Add camera pose vertices." << std::endl;
    std::default_random_engine generator;
    std::normal_distribution<double> camera_rotation_noise(0., 0.1);
    std::normal_distribution<double> camera_position_noise(0., 0.2);
    std::vector<std::shared_ptr<rootBA::VertexCameraPose<Scalar>>> cameraVertices;
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

        std::shared_ptr<rootBA::VertexCameraPose<Scalar>> cameraVertex(
            new rootBA::VertexCameraPose<Scalar>(cameras[i].qwc * noise_q_wc, cameras[i].twc + noise_t));
        cameraVertices.emplace_back(cameraVertex);
        problem.AddCamera(cameraVertex);
    }
    std::cout << "problem has " << problem.GetCamerasNum() << " cameras." << std::endl;

    // 第四步：构造特征点 position 节点，并添加到 problem 中
    std::cout << "\nStep 4: Add landmark position vertices." << std::endl;
    std::list<std::shared_ptr<rootBA::VertexLandmarkPosi<Scalar>>> landmarkVertices;
    size_t maxObserveNum = cameras.size() / 2;
    for (size_t i = 0; i < points.size(); ++i)
    {
        // 为初值添加随机噪声
        std::normal_distribution<double> landmark_position_noise(0., 0.5);
        rootBA::Vector3<Scalar> noise(landmark_position_noise(generator), landmark_position_noise(generator),
                                      landmark_position_noise(generator));
        std::shared_ptr<rootBA::VertexLandmarkPosi<Scalar>> landmarkVertex(
            new rootBA::VertexLandmarkPosi<Scalar>(points[i] + noise));
        // 构造观测
        std::unordered_map<size_t, std::shared_ptr<rootBA::CameraObserve<Scalar>>> observes;
        --maxObserveNum;
        for (size_t j = 0; j < cameras.size(); ++j)
        {
            std::shared_ptr<rootBA::HuberKernel<Scalar>> kernel(new rootBA::HuberKernel<Scalar>(1.0));
            std::shared_ptr<rootBA::CameraObserve<Scalar>> observe(new rootBA::CameraObserve<Scalar>(
                cameraVertices[j], cameras[j].featurePerId.find(i)->second.head<2>(), kernel));
            observes.insert(std::make_pair(j, observe));
            std::cout << j << ", ";
        }
        std::cout << std::endl;
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

    // 第八步：计算平均误差
    std::cout << "\nStep 8: Compute average residual." << std::endl;
    Scalar residual = 0.0;
    int cnt = 0;
    for (auto landmark : landmarkVertices)
    {
        rootBA::Vector3<Scalar> diff = points[cnt] - landmark->Get_p_w();
        residual += rootBA::ComputeTranslationMagnitude(diff);
        ++cnt;
    }
    std::cout << "  Average landmark position residual is " << residual / Scalar(cnt) << " m" << std::endl;

    residual = Scalar(0);
    for (size_t i = 0; i < cameraVertices.size(); ++i)
    {
        rootBA::Quaternion<Scalar> diff = cameras[i].qwc.inverse() * cameraVertices[i]->Get_q_wb();
        residual += rootBA::ComputeRotationMagnitude(diff);
    }
    std::cout << "  Average camera atitude residual is " << residual / Scalar(cameras.size()) << " rad, "
              << residual / Scalar(cameras.size()) * 57.29578049 << " deg" << std::endl;

    residual = Scalar(0);
    for (size_t i = 0; i < cameraVertices.size(); ++i)
    {
        rootBA::Vector3<Scalar> diff = cameras[i].twc - cameraVertices[i]->Get_t_wb();
        residual += rootBA::ComputeTranslationMagnitude(diff);
    }
    std::cout << "  Average camera position residual is " << residual / Scalar(cameras.size()) << " m" << std::endl;

    return 0;
}