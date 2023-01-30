#pragma once

#include <memory>
#include <Eigen/core>
#include "sophus/se3.hpp"
#include "opencv2/opencv.hpp"
#include <memory>

namespace slam_fe {

class Camera {
    public:
    typedef std::shared_ptr<Camera> Ptr;

    float fx_, fy_;
    float cx_, cy_;
    float depth_scale_;

    Camera() = default;
    Camera(std::string settingPath);
    Camera(float fx, float fy, float cx, float cy, float depth=0):
        fx_(fx), fy_(fy), cx_(fx), cy_(cy), depth_scale(depth) {}

    Vector3d world2camera(const Vector3d& p_w, const Sophus::SE3& T_c_w);
    cv::Point3d world2camera(cv::Point3d p_w, cv::Mat T_c_w);

    Vector3d camera2world(const Vector3d& p_c, const Sophus::SE3& T_c_w);
    cv::Point3d camera2world(cv::Point3d p_c, cv::Mat  T_w_c);
    
    Vector3d camera2pixel(const Vector3d& p_c);
    cv::Point2d camera2pixel(cv::point3d p_c);
    
    Vector3d pixel2camera(const Vector2d& p_p, double depth=1);
    cv::Point3d pixel2camera(cv::Point2d p_p, double depth = 1.0);

    Vector3d pixel2world(const Vector2d& p_p, 
            const Sophus::SE3& T_c_w, double depth=1);
    Vector2d world2pixel(const Vector3d& p_w, const SE3& T_c_w);
    
    // implement this later.
    Camera(const Camera&);
    Camera& operator=(const Camera&);
};

} // namespace slam_fe