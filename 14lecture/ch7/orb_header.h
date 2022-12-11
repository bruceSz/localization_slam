
#pragma once

#include <opencv2/opencv.hpp>

typedef std::vector<uint32_t> DescType; // Descriptor type



void computeORB(const cv::Mat& input, const std::vector<cv::KeyPoint>& kps, std::vector<DescType>& desc1);
void BfMatch( const std::vector<DescType>& desc1, const std::vector<DescType>& desc2, std::vector<cv::DMatch>& matches);
