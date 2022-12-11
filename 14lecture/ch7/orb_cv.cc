
#include <iostream>
//#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
//#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/legacy/constants_c.h>

using std::cout ;
using std::endl;
int main(int argc, char** argv) {

    if (argc != 3) {
        cout << "Usage: " << argv[0] << " left_img right_img" << endl;
        return -1;
    }

    cv::Mat img1 = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
    cv::Mat img2 = cv::imread(argv[2], CV_LOAD_IMAGE_COLOR);

    assert(img1.data != nullptr);
    assert(img2.data != nullptr);

    //cv::imshow("1 img", img1);
    
    //cv::imshow("2 img", img2);


    std::vector<cv::KeyPoint> kp1, kp2;
    cv::Mat desc1, desc2;

    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");


    // 1  detect oriented FAST.
    detector->detect(img1, kp1);
    detector->detect(img2, kp2);

    // 2 compute desctiptor.

    descriptor->compute(img1, kp1, desc1);
    descriptor->compute(img2, kp2, desc2);

    //3 draw or features.
    //Todo.

    cv::Mat outimg;
    cv::drawKeypoints(img1, kp1, outimg, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
    //cv::imshow("ORB features", outimg);

    cout << "orb features done. " << endl;


    std::vector<cv::DMatch> matches;

    matcher->match(desc1, desc2, matches);
    cout << "descriptor mathced." << std::endl;


    auto min_max = std::minmax_element(matches.begin(), matches.end(),
                            [](const cv::DMatch& m1, const cv::DMatch& m2){
                                return m1.distance < m2.distance;
                            });

    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;

    std::vector<cv::DMatch> good_matches;
    for (int i=0; i< desc1.rows; i++) {
        if (matches[i].distance <= std::max(2 * min_dist, 30.0)) {
            good_matches.push_back(matches[i]);
        }

    }

    cout << "draw matches. " << endl;

    cv::Mat img_match;
    cv::Mat img_good_match;

    cv::drawMatches(img1, kp1, img2, kp2, matches, img_match);
    cv::imshow("img matches", img_match);
    cv::drawMatches(img1, kp1, img2, kp2, good_matches, img_good_match);
    cv::imshow("img good matches", img_good_match);


    cv::waitKey(0);

    return 0;
}