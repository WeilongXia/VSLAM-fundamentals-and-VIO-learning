#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sophus/se3.h>
#include <string>
#include <unistd.h>
#include <vector>

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;

// path to compare file
string compare_file = "../data/compare.txt";

// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> Tg,
                    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> Te, const string &ID);

void ICP(vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> pts_g,
         vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> pts_e, Eigen::Matrix3d &R,
         Eigen::Vector3d &t);

int main(int argc, char **argv)
{

    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> Tg;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> Te;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> Tg_trans;

    vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> pts_g;
    vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> pts_e;

    /// implement pose reading code
    // start your code here (5~10 lines)

    ifstream compare_trajectory(compare_file);
    if (!compare_trajectory.is_open())
    {
        std::cout << "can not open files" << std::endl;
        return 0;
    }

    while (!compare_trajectory.eof())
    {
        double time_g, tx_g, ty_g, tz_g, qx_g, qy_g, qz_g, qw_g;
        double time_e, tx_e, ty_e, tz_e, qx_e, qy_e, qz_e, qw_e;
        compare_trajectory >> time_e >> tx_e >> ty_e >> tz_e >> qx_e >> qy_e >> qz_e >> qw_e >> time_g >> tx_g >>
            ty_g >> tz_g >> qx_g >> qy_g >> qz_g >> qw_g;
        Sophus::SE3 Tgi(Eigen::Quaterniond(qw_g, qx_g, qy_g, qz_g), Eigen::Vector3d(tx_g, ty_g, tz_g));
        Tg.emplace_back(Tgi);
        pts_g.emplace_back(Eigen::Vector3d(tx_g, ty_g, tz_g));
        Sophus::SE3 Tei(Eigen::Quaterniond(qw_e, qx_e, qy_e, qz_e), Eigen::Vector3d(tx_e, ty_e, tz_e));
        Te.emplace_back(Tei);
        pts_e.emplace_back(Eigen::Vector3d(tx_e, ty_e, tz_e));
    }

    // end your code here

    // Get Rotation and transpose through ICP method
    // Start code here
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    ICP(pts_g, pts_e, R, t);
    Sophus::SE3 Tge(R, t);
    for (auto T : Te)
    {
        Sophus::SE3 Tg_trans_i = Tge * T;
        Tg_trans.emplace_back(Tg_trans_i);
    }
    // End code here

    // draw trajectory in pangolin
    DrawTrajectory(Tg, Te, "Before Align");
    DrawTrajectory(Tg, Tg_trans, "After Align");
    return 0;
}

void ICP(vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> pts_g,
         vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> pts_e, Eigen::Matrix3d &R,
         Eigen::Vector3d &t)
{
    // center of mass
    Eigen::Vector3d p1, p2;
    int N = pts_g.size();
    std::cout << "N: " << N << std::endl;
    for (int i = 0; i < N; ++i)
    {
        p1 += pts_g[i];
        p2 += pts_e[i];
    }
    p1 = p1 / N;
    p2 = p2 / N;
    std::cout << "p1: " << p1.transpose() << std::endl;

    // remove the center
    vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> q1;
    // q1.reserve(1000);
    vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> q2;
    // q2.reserve(1000);
    for (int i = 0; i < N; ++i)
    {
        // q1[i] = pts_g[i] - p1;
        // q2[i] = pts_e[i] - p2;
        q1.emplace_back(pts_g[i] - p1);
        q2.emplace_back(pts_e[i] - p2);
    }

    // compute q1*q2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int i = 0; i < N; ++i)
    {
        W += q1[i] * q2[i].transpose();
    }
    std::cout << "W: " << W << std::endl;

    // Singular value decomposition of W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    R = U * (V.transpose());
    if (R.determinant() < 0)
    {
        R = -R;
    }
    t = p1 - R * p2;
}

/*******************************************************************************************/
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> Tg,
                    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> Te, const string &ID)
{
    if (Tg.empty() || Te.empty())
    {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    string WindowName = "Trajectory Viewer " + ID;

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind(WindowName, 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
                                      pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));

    pangolin::View &d_cam = pangolin::CreateDisplay()
                                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
                                .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < Tg.size() - 1; i++)
        {
            // glColor3f(1 - (float) i / groundtruth.size(), 0.0f, (float) i / groundtruth.size());
            glColor3f(255.0f, 0.0f, 0.0f);
            glBegin(GL_LINES);
            auto p1 = Tg[i], p2 = Tg[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        for (size_t i = 0; i < Te.size() - 1; i++)
        {
            // glColor3f(1 - (float) i / groundtruth.size(), 0.0f, (float) i / groundtruth.size());
            glColor3f(0.0f, 255.0f, 0.0f);
            glBegin(GL_LINES);
            auto p1 = Te[i], p2 = Te[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000); // sleep 5 ms
    }
}