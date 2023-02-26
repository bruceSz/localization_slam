#pragma once

#include <memory>

#include <opencv2/opencv.hpp>

#include "pose.h"

namespace zs {

class Image {
    public:
    Image() = default;

    Image(const cv::Mat& img) {
        img_ = img;
    }

    Image(cosnt cv::Mat& img, const zs::Pose3D& pose) {
        img_ = img;
        pose_ = pose;
    }

    ~Image() ;


    Image& operator=(const Image& rhs) {
        img_ = rhs.img_;
        pose_ = rhs.pose_;
        return *this;
    }

    Image(const Image& rhs) { 
        img_ = rhs.img_;
        pose_ = rhs.pose_;
    }

    cv::Mat& img() {
        return img_;
    }

    const zs::Pose3D& pose() {
        return pose_;
    }

private:
    cv::Mat img_;
    zs::Pose3D pose_;

public:

using ImagePtr = std::shared_ptr<Image>;

};



};