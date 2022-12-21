#pragma once
#include <vector>
#include <opencv2/opencv.hpp>


void find_feature_matches( const cv::Mat  & im1, const cv::Mat & im2,
                            std::vector<cv::KeyPoint> & kp1, std::vector<cv::KeyPoint> & kp2,
                            std::vector<cv::DMatch>& matches) ;


void pose_estimate_2dn2d(const std::vector<cv::KeyPoint> kp1, 
                        const std::vector<cv::KeyPoint> kp2,
                        const std::vector<cv::DMatch>& matches, cv::Mat& R, cv::Mat& t) ;


void triangulation(const std::vector<cv::KeyPoint>& kp1,
                    const std::vector<cv::KeyPoint>& kp2,
                    const std::vector<cv::DMatch>&  matches,
                    const cv::Mat& R,
                    const cv::Mat& t, 
                    std::vector<cv::Point3d>& points);

/// 作图用
inline cv::Scalar get_color(float depth) {
  float up_th = 50, low_th = 10, th_range = up_th - low_th;
  if (depth > up_th) depth = up_th;
  if (depth < low_th) depth = low_th;
  return cv::Scalar(255 * depth / th_range, 0, 255 * (1 - depth / th_range));
}

inline 
cv::Point2f pixel2cam(const cv::Point2d &p, const cv::Mat& k) {
    return cv::Point2f(
        (p.x - k.at<double>(0,2))/ k.at<double>(0,0),
        (p.y - k.at<double>(1,2))/ k.at<double>(1,1)
    );
}