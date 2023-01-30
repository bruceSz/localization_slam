#pragma once

#include "camera.h"
#include "sophus/se3.hpp"

#include <opencv2/core/core.hpp>


namespace slam_fe {

class MapPoint;
class Frame {
    public:
    typedef std::shared_ptr<Camera> Camera;
    unsigned long id_; // id of the frame.
    double  time_stamp;// when it is recorded
    Sophus::SE3  T_c_w; // transform from world to camera
    cv::Mat T_c_w_;
    Camera::Ptr  camera_;
    cv::Mat      color_, depth_;


    public:
    Frame();
    Frame(long id, double time_stamp =0,
        SE3 T_c_w = SE3(), Camera::Ptr camera = nullptr, 
        cv::Mat color_ = cv::Mat(), cv::Mat depth_ = cv::Mat());
    
    static Frame::Ptr createFrame();

    double findDepth(const cv::Keypoint& kp);

    Vector3d getCamCenter() const;

    bool isInFrame(const Vector3d& pt_world);


};

} // namespace of slam fe.