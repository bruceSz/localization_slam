
#include "common.h"
#include <algorithm>

#include <opencv2/imgcodecs/legacy/constants_c.h>


using cv::Mat;
using cv::KeyPoint;
using std::vector;
using std::cout ;
using std::endl;
using cv::FeatureDetector;
using cv::DescriptorExtractor;
using cv::DescriptorMatcher;
using cv::Ptr;
using cv::DMatch;
using cv::ORB;
using cv::Point2f;
using cv::recoverPose;
using cv::findEssentialMat;


void pose_estimate_2dn2d(const std::vector<cv::KeyPoint> kp1, 
                        const std::vector<cv::KeyPoint> kp2,
                        const std::vector<cv::DMatch>& matches,
                        cv::Mat & R, cv::Mat& t)  {
    Mat K = (cv::Mat_<double>(3,3) << 520.9, 0,325.1, 0 , 521.0, 249.7 , 0,0,1 );

    vector<Point2f> pt1;
    vector<Point2f> pt2;

    for(int i=0 ; i< matches.size(); i++) {
        pt1.push_back(kp1[matches[i].queryIdx].pt);
        pt2.push_back(kp2[matches[i].trainIdx].pt);
    }

    Point2f principal_point(325.1, 249.7);

    int focal = 521;

    Mat essential;
    essential = cv::findEssentialMat(pt1, pt2, focal, principal_point);

    recoverPose(essential, pt1, pt2, R, t, focal, principal_point);

}

void find_feature_matches( const Mat  & im1, const Mat & im2,
                            std::vector<KeyPoint> & kp1, std::vector<KeyPoint> & kp2,
                            std::vector<DMatch>& matches)  {
    Mat desc1, desc2;

    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();


    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    detector->detect(im1, kp1);
    detector->detect(im2, kp2);

    descriptor->compute(im1, kp1, desc1);
    descriptor->compute(im2, kp2, desc2);

    vector<DMatch> match;

    matcher->match(desc1, desc2, match);

    double min_dist = 1000, max_dist = 0;

    for(int i=0; i< desc1.rows; i++) {
        double dist = match[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }
    cout << " max match dist: " << max_dist << endl;
    cout << "min match dist: " << min_dist << endl;

    for(int i=0 ; i< desc1.rows; i++) {
        if (match[i].distance <= std::max(2*min_dist, 30.0)) {
            matches.push_back(match[i]);
        }
    }
    return;
}




void triangulation(const std::vector<cv::KeyPoint>& kp1,
                    const std::vector<cv::KeyPoint>& kp2,
                    const std::vector<cv::DMatch>&  matches,
                    const cv::Mat& R,
                    const cv::Mat& t, 
                    std::vector<cv::Point3d>& points) {
    cout << "enter into triangulation" << endl;
    Mat T1 = (cv::Mat_<float>(3,4) << 
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0);
    Mat T2 = (cv::Mat_<float>(3,4) <<
        R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
        R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0));
    cv::Mat k = (cv::Mat_<double>(3,3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    
    vector<Point2f> pt1, pt2;
    // pt in camera coordinate with z == 1.
    for(auto m: matches) {
        pt1.push_back(pixel2cam(kp1[m.queryIdx].pt, k));
        pt2.push_back(pixel2cam(kp2[m.trainIdx].pt, k));
    }

    cout << "matches pts gathered." << endl;
    Mat pts_4d;
    cv::triangulatePoints(T1 , T2, pt1, pt2, pts_4d);
    for(int i =0; i< pts_4d.cols; i++) {
        Mat x = pts_4d.col(i);
        x /= x.at<float>(3,0);
        cout <<" (3,0) is: " << x.at<float>(3,0);
        cv::Point3d p(
            x.at<float>(0,0),
            x.at<float>(1,0),
            x.at<float>(2,0)
        );
        points.push_back(p);
    }

    return;
}