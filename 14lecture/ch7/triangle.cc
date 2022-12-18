
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

    return 0;
}