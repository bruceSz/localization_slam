#pragma once
#include <vector>
#include <opencv2/opencv.hpp>


void find_feature_matches( const cv::Mat  & im1, const cv::Mat & im2,
                            std::vector<cv::KeyPoint> & kp1, std::vector<cv::KeyPoint> & kp2,
                            std::vector<cv::DMatch>& matches) ;


void pose_estimate_2dn2d(const std::vector<cv::KeyPoint> kp1, 
                        const std::vector<cv::KeyPoint> kp2,
                        const std::vector<cv::DMatch>& matches, cv::Mat& R, cv::Mat& t) ;