
#include <vector>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs/legacy/constants_c.h>

#include "common.h"

using std::cout ;
using std::endl;

using cv::Mat;
using cv::imread;
using cv::DMatch;
using cv::KeyPoint;
using cv::Point2d;

using std::vector;

int main(int argc, char** argv) {
    if (argc != 3) {
        cout << "Usage: " << argv[0] << " img1 img2." << endl;
        return -1;
    }

    Mat im1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat im2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);

    vector<KeyPoint> kp1, kp2;
    vector<DMatch> matches;


    find_feature_matches(im1, im2, kp1, kp2, matches);

    cout << "total features: "  << matches.size() << std::endl;


    Mat R, t;
    pose_estimate_2dn2d(kp1, kp2, matches, R, t);


    // verify t^R*scale

    Mat t_x = (cv::Mat_<double>(3,3) <<
                0, -t.at<double>(2.0), t.at<double>(1,0),
                t.at<double>(2,0), 0, -t.at<double>(0,0),
                -t.at<double>(1,0), t.at<double>(0,0), 0
                );
    cout << "t^R=" << t_x *R << endl;


    Mat k = (cv::Mat_<double>(3,3) <<
         520.9, 0, 325.1 ,
         0,521.0,249.7,
          0,0,1);

    for (DMatch m: matches) {
        Point2d pt1 = pixel2cam(kp1[m.queryIdx].pt, k);
        Mat y1 = (cv::Mat_<double>(3,1) << pt1.x, pt1.y, 1);

        Point2d pt2 = pixel2cam(kp2[m.trainIdx].pt,k);
        Mat y2 = (cv::Mat_<double>(3,1) << pt2.x,pt2.y, 1);

        Mat d = y2.t() * t_x * R * y1;
        cout << "epipolar constraint: " << d << endl;
    }

    return 0;

}