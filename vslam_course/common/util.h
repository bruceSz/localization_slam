#pragma once

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

#include "point.h"
#include "camera.h"
#include "pose.h"


namespace zs {

template<typename ValType, typename DistType>
ValType bilinear(const ValType& val1, const ValType& val2, 
    const ValType& val3, const ValType& val4,
    const DistType& dist1, const DistType& dist2, const DistType& dist3, const DistType& dist4);


zs::Point2D distort(const zs::Point2D& point, const zs::Camera::CameraPtr cam);


cv::Mat undistort(const cv::Mat& img, const Camera::CameraPtr cam);

zs::Point2D project(const zs::Point3D& pt_cam, const Camera::CameraPtr cam);

zs::Point2D project(const zs::Point3D pt_world, const Camera::CameraPtr cam, const zs::Pose3D pose);

zs::Point3D unproject(const zs::Point2D p, const Camera::CameraPtr cam);

zs::Point3D unproject(const zs::Point2D p, const Camera::CameraPtr cam, const zs::Pose3D pose);

zs::Point3D transform(const zs::Point3D pt, const zs::Pose3D pose);


/**
 * 合并图像
 * @param imgs 图像
 * @param w    图像缩放为指定大小，宽
 * @param h    图像缩放为指定大小，高
*/
cv::Mat mergeImage(const std::vector<cv::Mat>& imgs, int w, int h);


/**
 * 从文本文件加载矩阵，仅限于空格隔开
 * @param file 文件路径
 * @param row  行数
 * @param col  列数
*/
Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> load_matrix(const std::string& file, int row, int col);



inline std::string folder_and_slash(const std::string& folder) {
    size_t len = folder.length();
    if(len > 0 && folder[len-1] != '/') {
        return folder + "/";
    }
    return folder;
}



} // namespace of zs