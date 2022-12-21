
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "common.h"

using cv::imread;
using cv::Mat;
using std::endl;
using std::cout;
using cv::KeyPoint;
using cv::DMatch;
using cv::ORB;
using cv::Point3d;
using std::vector;


int main(int argc, char** argv) {

    if (argc != 3) {
        cout << "Usage: " << argv[0] << " img1 img2" << endl;
        return -1;
    }

    Mat im1 = imread(argv[1]);
    Mat im2 = imread(argv[2]);

    std::vector<KeyPoint> kp1, kp2;
    std::vector<DMatch> matches;

    find_feature_matches(im1, im2, kp1, kp2, matches);
    cout << " together find: " << matches.size() << endl;

    vector<Point3d> points;
    Mat R, t;

    pose_estimate_2dn2d(kp1, kp2, matches, R, t);

    cout << "R is: " << R << endl;
    cout << "t is : " << t << endl;
    triangulation(kp1, kp2, matches, R,t,points);


    cout << "triangulation: " << points.size() << endl;

    // verify reprojection error
    Mat K = (cv::Mat_<double>(3,3) << 520.9, 0, 325.1 , 0, 521.0, 249.7, 0,0,1);

    Mat im1_plot = im1.clone();
    cv::Mat im2_plot = im2.clone();

    for(size_t i=0 ;i< matches.size(); i++) {
        float depth = points[i].z;
        //cout << " depth: " << depth << endl;
        cv::Point2d pt1_cam = pixel2cam(kp1[matches[i].queryIdx].pt, K);
        cv::circle(im1_plot, kp1[matches[i].queryIdx].pt, 2, get_color(depth),2);

        Mat pt2_trans = R*(cv::Mat_<double>(3,1) << points[i].x,points[i].y, points[i].z) + t;
        float depth2 = pt2_trans.at<double>(2,0);
        cv::circle(im2_plot, kp2[matches[i].trainIdx].pt, 2, get_color(depth2),2);
    }

    cv::imshow("img1", im1_plot);
    cv::imshow("img2", im2_plot);
    cv::waitKey();

    return 0;
}