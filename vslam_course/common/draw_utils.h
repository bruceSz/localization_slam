#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

namespace zs {

cv::Mat drawPoint(const cv::Mat &img, const Eigen::Matrix<double,Eigen::Dynamic,2>& pxs, 
                    DrawType type, , const cv::Scalar &color);

template<class PointType>
cv::Mat drawPoint(const cv:Mat& img, const std::vector<PointType>& pxs, DrawType type, const cv::Scala& color) {
    cv::Mat img_res(img.rows, img.cols, CV_8UC3);
    if (img.channels() == 1) {
        std::vector<cv::Mat> channels(3, img);
        cv::merge(channels, img_res);
    } else {
        img.copyTo(img_res);
    }

    for(int i=0; i< pxs.size(); i++) {
        double x = pxs[i].x;
        double y = pxs[i].y;

        if (x< 0 || x>img.cols || y< 0 || y>img.rows) {
            continue;
        }

        if (type == DrawType::CIRCLE) {
            cv::circle(img_res, cv::Point(x,y), 4, color);
        } else if (type == DrawType::POINT) {
            cv::circle(img_res, cv::Point(x,y), 2, color, -1);
        } else if (type == DrawType::X) {
            double x_l = x-3;
            double x_r = x+3;
            double y_l = y-3;
            double y_r = y+3;

            if (x_l < 0) x_l = 0;
            if (x_r>=img.cols) x_r = img.cols;
            if (y_l < 0) y_l = 0;
            if (y_r>=img.rows) y_r = img.rows;

            cv::line(img_res, cv::Point(x_l,y_l), cv::Point(x_r,y_r), color);
            cv::line(img_res, cv::Point(x_l,y_r), cv::Point(x_r,y_l), color);
        } else if (type == DrawType::RECT) {
            double x_l = x-3;
            double x_r = x+3;
            double y_l = y-3;
            double y_r = y+3;
            if (x_l < 0) x_l = 0;
            if (x_r>=img.cols) x_r = img.cols;
            if (y_l < 0) y_l = 0;
            if (y_r>=img.rows) y_r = img.rows;
            cv::rectangle(img_res, cv::Rect(x_l, y_l, x_r - x_l, y_r - y_l), color, 2);
        }
    }

    return img_res;
}

}