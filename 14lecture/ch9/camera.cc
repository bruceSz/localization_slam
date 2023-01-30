#include "camera.h"

Camera::Camera(std::string settingPath) {
    cv::FileStorage setting(settingPath, cv::FileStorage::READ);
    fx_ = setting["Camera.fx"];
    fy_ = setting["Camera.fy"];
    cx_ = setting["Camera.cx"];
    cy_ = setting["Camera.cy"];
    depth_ = setting["Camera.depth"];
}

Vector3d Camera::world2camera(const Vector3d& p_w, 
    const Sophus::SE3& T_c_w) { 
        return T_c_w * p_w;

}

Vector3d Camera::camera2world(const Vector3d& p_c, const Sophus::SE3& T_c_w) { 
    return T_c_w.inverse() * p_c;
}

cv::Point3d camera2world(cv::Point3d p_c, cv::Mat  T_w_c) {
    cv::Mat pts3_c(p_c);
    cv::Mat pts4_c(4,1, CV_64FC1);

    pts3_c.copyTo(pts4_c(cv::Rect(0, 0, 1, 3)));
    pts4_c.at<double>(3,0) = 1.0;
    
    cv::Mat pts_conv = T_w_c * pts4_c;
    return cv::Point3d(pts_conv(cv::Rect(0,0,1,3)));
}

cv::Point2d camera2pixel(cv::point3d p_c, double depth) {
    cv::Mat pts3_c(p_c);
    return cv::Point2d(
        fx_ * pts3_c.at<double>(0,0)/pts3_c.at<double>(2,0) + cx_,
        fy_ * pts3_c.at<double>(1,0)/pts3_c.at<double>(2,0) + cy_
    );
}

Vector2d Camera::camera2pixel(const Vector3d& p_c) {
    return Vector2d(
        fx_ * p_x(0,0) / p_c(2,0) + cx_,
        fy_ * p_c(1,0) / p_c(2,1) + cy_
    );

}

cv::Point3d pixel2camera(cv::Point2d p_p, double depth = 1.0) {
    cv::Mat pts_p(p_p);
    return cv::Point3d(
        (pts_p.at<double>(0,0) - cx_) * depth / fx_,
        (pts_p.at<double>(1,0) - cy_) * depth / fy_,
        depth
    );
}


Vector2d Camera::world2pixel(const Vector3d& p_w, const Sophus::SE3& T_c_w) {
    camera2pixel(world2camera(p_w, T_c_w));
}

Vector3d Camera::pixel2world(const Vector2d& p_p, const Sophus::SE3& T_c_w, double depth) {
    return camera2world(pixel2camera(p_p, depth), T_c_w);
}

