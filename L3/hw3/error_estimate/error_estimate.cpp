#include <sophus/se3.h>
#include <Eigen/Geometry>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <unistd.h>
#include <iterator>

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;

// path to groundtruth file
string groundtruth_file = "../data/groundtruth.txt";
// path to estimated file
string estimated_file = "../data/estimated.txt";

// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> groundtruth,
                    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> estimated);

int main(int argc, char **argv) {

    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> groundtruth_poses;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> estimated_poses;

    /// implement pose reading code
    // start your code here (5~10 lines)

    ifstream groundtruth_trajectory(groundtruth_file);
    ifstream estimated_trajectory(estimated_file);
    if(!groundtruth_trajectory.is_open() || !estimated_trajectory.is_open())
    {
        std::cout << "can not open files" << std::endl;
        return 0;
    }

    while(!groundtruth_trajectory.eof())
    {
        double time, tx, ty, tz, qx, qy, qz, qw;
        groundtruth_trajectory >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Sophus::SE3 Twc(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz));
        groundtruth_poses.emplace_back(Twc);
    }

    while(!estimated_trajectory.eof())
    {
        double time, tx, ty, tz, qx, qy, qz, qw;
        estimated_trajectory >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Sophus::SE3 Twc(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz));
        estimated_poses.emplace_back(Twc);
    }

    // end your code here

    // traverse groundtruth and estimated
    // calculate errors and accumulate to get RMSE
    float acc_error = 0.0;
    float rmse = 0.0;

    for(int i = 0; i < estimated_poses.size(); ++i)
    {
        Sophus::SE3 error_SE3;
        error_SE3 = groundtruth_poses[i].inverse() * estimated_poses[i];
        acc_error += error_SE3.log().squaredNorm();
    }

    rmse = sqrt(acc_error / estimated_poses.size());

    std::cout << "RMSE is: " << rmse << std::endl;

    // draw trajectory in pangolin
    DrawTrajectory(groundtruth_poses, estimated_poses);
    return 0;
}

/*******************************************************************************************/
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> groundtruth, 
                    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> estimated) {
    if (groundtruth.empty() || estimated.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Error Analysis", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < groundtruth.size() - 1; i++) {
            // glColor3f(1 - (float) i / groundtruth.size(), 0.0f, (float) i / groundtruth.size());
            glColor3f(255.0f, 0.0f, 0.0f);
            glBegin(GL_LINES);
            auto p1 = groundtruth[i], p2 = groundtruth[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        for (size_t i = 0; i < estimated.size() - 1; i++) {
            // glColor3f(1 - (float) i / groundtruth.size(), 0.0f, (float) i / groundtruth.size());
            glColor3f(0.0f, 255.0f, 0.0f);
            glBegin(GL_LINES);
            auto p1 = estimated[i], p2 = estimated[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}