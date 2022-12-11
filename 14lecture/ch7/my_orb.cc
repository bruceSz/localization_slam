#include <iostream>
#include <vector>
#include "orb_header.h"

#include <opencv2/opencv.hpp>

using std::cout;
using std::endl;



int main(int argc, char** argv) {

    if (argc != 3) {
        cout << "Usage: " << argv[0] << " img1 img2 ..." << endl;
    }

    cv::Mat img1 = cv::imread(argv[1]);
    cv::Mat img2 = cv::imread(argv[2]);
    assert(img1.data != nullptr);
    assert(img2.data != nullptr);

    //
    std::vector<cv::KeyPoint> kp1;
    cv::FAST(img1, kp1, 40);
    std::vector<DescType> desc1;
    computeORB(img1, kp1, desc1);

    //
    std::vector<cv::KeyPoint> kp2;
    cv::FAST(img2, kp2, 40);
    std::vector<DescType> desc2;
    computeORB(img2, kp2, desc2);

    //
    std::vector<cv::DMatch> matches;
    BfMatch(desc1, desc2,matches);
    cout <<  "matches: " << matches.size() << endl;


    cv::Mat out_img;

    cv::drawMatches(img1, kp1, img2, kp2, matches, out_img);
    cv::imshow("matches", out_img);
    
    cv::waitKey(0);


    return 0;
}