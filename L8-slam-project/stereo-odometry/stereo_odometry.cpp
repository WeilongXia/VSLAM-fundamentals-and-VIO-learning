/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
 * For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * This file is a simple interface which can use Kitti dataset to
 * test our stereo visual odometry algorithm.
 */

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

void LoadImages(const std::string &strPathToSequence, std::vector<std::string> &vstrImageLeft,
                std::vector<std::string> &vstrImageRight, std::vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if (argc != 4)
    {
        std::cerr << std::endl << "Usage: ./stereo_odometry path_to_settings path_to_sequence" << std::endl;
        return 1;
    }

    // Retrieve paths to iamges
    std::vector<std::string> vstrImageLeft;
    std::vector<std::string> vstrImageRight;
    std::vector<double> vTimestamps;
    LoadImages(std::string(argv[2]), vstrImageLeft, vstrImageRight, vTimestamps);

    const int nImages = vTimestamps.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames
    Stereo_SLAM::System SLAM(argv[1], true);

    std::cout << std::endl << "*************" << std::endl;
    std::cout << "Start processing sequence ..." << std::endl;
    std::cout << "Images in the sequence: " << nImages << std::endl << std::endl;

    // Main loop
    cv::Mat imLeft, imRight;
    for (int i = 0; i < nImages; ++i)
    {
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[i], cv::IMREAD_UNCHANGED);
        imRight = cv::imread(vstrImageRight[i], cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[i];

        if (imLeft.empty())
        {
            std::cerr << std::endl << "Failed to load image at: " << std::string(vstrImageLeft[i]) << std::endl;
            return 1;
        }

        SLAM.Track(imLeft, imRight, tframe);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveTrajectory("CameraTrajectory.txt");

    return 0;
}

void LoadImages(const std::string &strPathToSequence, std::vector<std::string> &vstrImageLeft,
                std::vector<std::string> &vstrImageRight, std::vector<double> &vTimestamps)
{
    std::ifstream fTimes;
    std::string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathToSequence.c_str());
    while (!fTimes.eof())
    {
        std::string s;
        getline(fTimes, s);
        if (!s.empty())
        {
            std::stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimeStamps.push_back(t);
        }
    }

    std::string strPrefixLeft = strPathToSequence + "/image_0/";
    std::string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for (int i = 0; i < nTimes; ++i)
    {
        std::stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}
