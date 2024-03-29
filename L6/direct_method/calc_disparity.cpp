#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>
#include <sophus/se3.hpp>
#include <string>
#include <vector>

using namespace std;
using namespace cv;

// Camera intrinsics
// 内参
double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
// 基线
double baseline = 0.573;

typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;

// TODO implement this funciton
/**
 * single level optical flow
 * @param [in] img1 the first image
 * @param [in] img2 the second image
 * @param [in] kp1 keypoints in img1
 * @param [in|out] kp2 keypoints in img2, if empty, use initial guess in kp1
 * @param [out] success true if a keypoint is tracked successfully
 * @param [in] inverse use inverse formulation?
 */
void OpticalFlowSingleLevel(const Mat &img1, const Mat &img2, const vector<KeyPoint> &kp1, vector<KeyPoint> &kp2,
                            vector<bool> &success, bool inverse = false);

// TODO implement this funciton
/**
 * multi level optical flow, scale of pyramid is set to 2 by default
 * the image pyramid will be create inside the function
 * @param [in] img1 the first pyramid
 * @param [in] img2 the second pyramid
 * @param [in] kp1 keypoints in img1
 * @param [out] kp2 keypoints in img2
 * @param [out] success true if a keypoint is tracked successfully
 * @param [in] inverse set true to enable inverse formulation
 */
void OpticalFlowMultiLevel(const Mat &img1, const Mat &img2, const vector<KeyPoint> &kp1, vector<KeyPoint> &kp2,
                           vector<bool> &success, bool inverse = false);

/**
 * get a gray scale value from reference image (bi-linear interpolated)
 * @param img
 * @param x
 * @param y
 * @return
 */
inline float GetPixelValue(const cv::Mat &img, float x, float y)
{
    uchar *data = &img.data[int(y) * img.step + int(x)];
    float xx = x - floor(x);
    float yy = y - floor(y);
    return float((1 - xx) * (1 - yy) * data[0] + xx * (1 - yy) * data[1] + (1 - xx) * yy * data[img.step] +
                 xx * yy * data[img.step + 1]);
}

int main()
{
    // load images
    cv::Mat leftImg, rightImg, disparityImg;
    leftImg = imread("../pics/left.png", 0);
    rightImg = imread("../pics/right.png", 0);
    disparityImg = imread("../pics/disparity.png", 0);

    // detect keypoints
    std::vector<KeyPoint> KeyPoints1;
    std::vector<KeyPoint> KeyPoints2;
    cv::Ptr<GFTTDetector> gftt = GFTTDetector::create();
    gftt->detect(leftImg, KeyPoints1);

    // VecVector2d px_left, px_right;
    // for (int i = 0; i < KeyPoints1.size(); ++i)
    // {
    //     px_left.emplace_back(Eigen::Vector2d(KeyPoints1[i].pt.x, KeyPoints1[i].pt.y));
    // }
    // px_right.resize(px_left.size());

    std::vector<bool> success;

    // calculate disparities iteratively
    int err = 0;
    int num = 0;
    OpticalFlowMultiLevel(leftImg, rightImg, KeyPoints1, KeyPoints2, success, false);

    for (int i = 0; i < KeyPoints1.size(); ++i)
    {
        if (success[i] == false)
        {
            continue;
        }
        double disparity1 = KeyPoints1[i].pt.x - KeyPoints2[i].pt.x;
        std::cout << "the calculated disparity is " << disparity1 << "    ";
        double disparity2 = disparityImg.at<uchar>(KeyPoints2[i].pt.y, KeyPoints1[i].pt.x);
        std::cout << "and the ground_truth disparity is " << disparity2 << std::endl;
        num++;
        err += abs(disparity1 - disparity2);
    }

    std::cout << "the average error is " << err / num << std::endl;
}

void OpticalFlowSingleLevel(const Mat &img1, const Mat &img2, const vector<KeyPoint> &kp1, vector<KeyPoint> &kp2,
                            vector<bool> &success, bool inverse)
{

    // parameters
    int half_patch_size = 4;
    int iterations = 10;
    bool have_initial = !kp2.empty();

    for (size_t i = 0; i < kp1.size(); i++)
    {
        auto kp = kp1[i];
        double dx = 0, dy = 0; // dx,dy need to be estimated
        if (have_initial)
        {
            dx = kp2[i].pt.x - kp.pt.x;
            dy = kp2[i].pt.y - kp.pt.y;
        }

        double cost = 0, lastCost = 0;
        bool succ = true; // indicate if this point succeeded

        // Gauss-Newton iterations
        for (int iter = 0; iter < iterations; iter++)
        {
            Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
            Eigen::Vector2d b = Eigen::Vector2d::Zero();
            Eigen::Vector2d J;

            if (inverse == false)
            {
                H = Eigen::Matrix2d::Zero();
                b = Eigen::Vector2d::Zero();
            }
            else
            {
                // only reset b
                b = Eigen::Vector2d::Zero();
            }
            cost = 0;

            if (kp.pt.x + dx <= half_patch_size || kp.pt.x + dx >= img1.cols - half_patch_size ||
                kp.pt.y + dy <= half_patch_size || kp.pt.y + dy >= img1.rows - half_patch_size)
            { // go outside
                succ = false;
                break;
            }

            // compute cost and jacobian
            for (int x = -half_patch_size; x < half_patch_size; x++)
                for (int y = -half_patch_size; y < half_patch_size; y++)
                {

                    // TODO START YOUR CODE HERE (~8 lines)
                    double error = GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y) -
                                   GetPixelValue(img2, kp.pt.x + x + dx, kp.pt.y + y + dy);
                    if (inverse == false)
                    {
                        // Forward Jacobian
                        // 中值梯度
                        J = -1.0 * Eigen::Vector2d(0.5 * (GetPixelValue(img2, kp.pt.x + dx + x + 1, kp.pt.y + y + dy) -
                                                          GetPixelValue(img2, kp.pt.x + dx + x - 1, kp.pt.y + y + dy)),
                                                   0.5 * (GetPixelValue(img2, kp.pt.x + dx + x, kp.pt.y + y + dy + 1) -
                                                          GetPixelValue(img2, kp.pt.x + dx + x, kp.pt.y + y + dy - 1)));
                    }
                    else if (iter == 0)
                    {
                        // Inverse Jacobian
                        J = -1.0 * Eigen::Vector2d(0.5 * (GetPixelValue(img1, kp.pt.x + x + 1, kp.pt.y + y) -
                                                          GetPixelValue(img1, kp.pt.x + x - 1, kp.pt.y + y)),
                                                   0.5 * (GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y + 1) -
                                                          GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y - 1)));
                        // NOTE this J does not change when dx, dy is updated, so we can store it and only compute error
                    }

                    // compute H, b and set cost;
                    b += -error * J;
                    cost += error * error;
                    if (inverse == false || iter == 0)
                    {
                        // also update H
                        H += J * J.transpose();
                    }
                    // TODO END YOUR CODE HERE
                }

            // compute update
            // TODO START YOUR CODE HERE (~1 lines)
            Eigen::Vector2d update = H.ldlt().solve(b);
            // TODO END YOUR CODE HERE

            if (isnan(update[0]))
            {
                // sometimes occurred when we have a black or white patch and H is irreversible
                cout << "update is nan" << endl;
                succ = false;
                break;
            }
            if (iter > 0 && cost > lastCost)
            {
                cout << "cost increased: " << cost << ", " << lastCost << endl;
                break;
            }

            // update dx, dy
            dx += update[0];
            dy += update[1];
            lastCost = cost;
            succ = true;
        }

        success.push_back(succ);

        // set kp2
        if (have_initial)
        {
            kp2[i].pt = kp.pt + Point2f(dx, dy);
        }
        else
        {
            KeyPoint tracked = kp;
            tracked.pt += cv::Point2f(dx, dy);
            kp2.push_back(tracked);
        }
    }
}

void OpticalFlowMultiLevel(const Mat &img1, const Mat &img2, const vector<KeyPoint> &kp1, vector<KeyPoint> &kp2,
                           vector<bool> &success, bool inverse)
{

    // parameters
    int pyramids = 4;
    double pyramid_scale = 0.5;
    double scales[] = {1.0, 0.5, 0.25, 0.125};

    // create pyramids
    vector<Mat> pyr1, pyr2; // image pyramids
    // TODO START YOUR CODE HERE (~8 lines)
    for (int i = 0; i < pyramids; i++)
    {
        if (i == 0)
        {
            pyr1.push_back(img1);
            pyr2.push_back(img2);
        }
        else
        {
            Mat img1_pyr, img2_pyr;
            cv::resize(pyr1[i - 1], img1_pyr,
                       cv::Size(pyr1[i - 1].cols * pyramid_scale, pyr1[i - 1].rows * pyramid_scale));
            cv::resize(pyr2[i - 1], img2_pyr,
                       cv::Size(pyr2[i - 1].cols * pyramid_scale, pyr2[i - 1].rows * pyramid_scale));
            pyr1.push_back(img1_pyr);
            pyr2.push_back(img2_pyr);
        }
    }
    // TODO END YOUR CODE HERE

    // coarse-to-fine LK tracking in pyramids

    // TODO START YOUR CODE HERE
    vector<KeyPoint> kp1_pyr, kp2_pyr;
    for (auto &kp : kp1)
    {
        auto kp_top = kp;
        kp_top.pt *= scales[pyramids - 1];
        kp1_pyr.push_back(kp_top);
        kp2_pyr.push_back(kp_top);
    }

    for (int level = pyramids - 1; level >= 0; level--)
    {
        // from coarse to fine
        success.clear();
        OpticalFlowSingleLevel(pyr1[level], pyr2[level], kp1_pyr, kp2_pyr, success, inverse);

        if (level > 0)
        {
            for (auto &kp : kp1_pyr)
            {
                kp.pt /= pyramid_scale;
            }

            for (auto &kp : kp2_pyr)
            {
                kp.pt /= pyramid_scale;
            }
        }
    }

    for (auto &kp : kp2_pyr)
    {
        kp2.push_back(kp);
    }

    // TODO END YOUR CODE HERE
    // don't forget to set the results into kp2
}