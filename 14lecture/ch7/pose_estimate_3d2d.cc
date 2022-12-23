
#include<iostream>

using std::cout;
using std::endl;

#include <opencv2/opencv.hpp>

#include "common.h"

using std::cout ;
using std::endl;

using cv::Mat;
using cv::imread;
using cv::DMatch;
using cv::KeyPoint;
using cv::Point2d;
using cv::Point3f;
using cv::Point2f;

using std::vector;

int main(int argc, char** argv) {

    if (argc != 5 ) {
        cout << "usage: " << argv[0] << " img1, img2, depth1, depth2" << endl;
        return 1;
    }

    Mat im1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat im2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);

    vector<KeyPoint> kp1,kp2;
    vector<DMatch> matches;

    find_feature_matches(im1, im2, kp1, kp2, matches);

    cout << "total matches: " << matches.size() << std::endl;

    Mat d1 = imread(argv[3], CV_LOAD_IMAGE_UNCHANGED);

    Mat k = (cv::Mat_<double>(3,3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );

    vector<Point3f> pt_3d;
    vector<Point2f> pt_2d;

    for (auto m: matches) {
        ushort d = d1.ptr<unsigned int>(int(kp1[m.queryIdx].pt.y))[int(kp1[m.queryIdx].pt.x)];
        if (d ==0) 
            continue;
        float dd = d/ 5000.0;

        Point2d p1 = pixel2cam(kp1[m.queryIdx].pt,k);
        pt_3d.push_back(Point3f(p1.x * dd, p1.y*dd, dd));
        pt_2d.push_back(kp1[m.queryIdx].pt);

    }

    cout << "3d-2d pairs: " << pt_3d.size() << std::endl;

    Mat r, t;

    cv::solvePnP(pt_3d, pt_2d, k, Mat(), r, t, false);

    Mat R;
    cv::Rodrigues(r, R);

    cout << "R="<< R << endl;
    cout << "t=" << t << endl;

    bundleAdjustment(pt_3d, pt_2d, k, R, t);













    return 0;
}