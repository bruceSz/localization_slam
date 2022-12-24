#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

#include <opencv2/imgcodecs/legacy/constants_c.h>

#include "common.h"


using std::vector;
using std::cout;
using std::endl;

using cv::KeyPoint;
using cv::Mat;
using cv::imread;
using cv::Point3f;
using cv::Point2d;
using cv::DMatch;

int main(int argc, char** argv) {

    if(argc != 5) {
        cout << "Usage: " << argv[0] << " im1 im2 depth1 depth2" << endl;
        return 1;
    }

    Mat im1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat im2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);

    vector<KeyPoint> kp1, kp2;
    vector<DMatch> matches;

    find_feature_matches(im1, im2, kp1, kp2,matches);
    cout << "total matches: " << matches.size() << endl;



    Mat dep1 = imread(argv[3], CV_LOAD_IMAGE_UNCHANGED);
    Mat dep2 = imread(argv[4], CV_LOAD_IMAGE_UNCHANGED);

    Mat K = ( cv::Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );

    vector<Point3f> pt1, pt2;
    for (auto m: matches) {
        ushort d1 = dep1.ptr<unsigned short>(int(kp1[m.queryIdx].pt.y))[int(kp1[m.queryIdx].pt.x)];
        ushort d2 = dep2.ptr<unsigned short>(int(kp2[m.trainIdx].pt.y))[int(kp2[m.trainIdx].pt.x)];
        if( d1 ==0 || d2 ==0 )
            continue;
        Point2d p1 = pixel2cam(kp1[m.queryIdx].pt, K);
        Point2d p2 = pixel2cam(kp2[m.trainIdx].pt,K);
        float dd1 = float((d1))/5000.0;
        float dd2 = float(d2)/5000.0;
        pt1.push_back(Point3f(p1.x*dd1, p1.y*dd1, dd1));
        pt2.push_back(Point3f(p2.x*dd2, p2.y*dd2, dd2));
    }


    cout << "3-3 pairs:" << pt1.size() << endl;

    Mat R, t;
    pose_estimation_3d3d(pt1, pt2, R, t);

    cout << "ICP via svd results:" << endl;


    cout << "R=" << R << endl;
    cout << "t=" << t << endl;
    cout << "R_inv=" << R.t() << endl;
    cout << "t_inv=" << -R.t() * t << endl;

    cout << "ba results: " << endl;

    OptICP(pt1, pt2, R, t);

    for(int i=0; i< 5; i++) {
        cout << "p1 = " << pt1[i] << endl;
        cout << "p2= " << pt2[i] << endl;
        cout << "(R*p2 + t) = " << 
            R * (cv::Mat_<double>(3,1) << pt2[i].x, pt2[i].y, pt2[i].z) + t << endl;
        
    }

}