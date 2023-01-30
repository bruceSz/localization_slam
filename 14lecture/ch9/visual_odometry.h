#pragma once

#include "map.h"
#include <opencv2/opencv.hpp>

namespace slam_fe{

class Viewer;
class VisualOdometry {
    public:
    enum VOState {
        INIT = -1,
        OK = 0, 
        LOST
    };

    VOState state_;

    // map
    Map map_;

    Frame* ref_;
    Frame* curr_;

    std::vector<Frame*>  allFrame;

    Viewer* mpViewer;
    Camera* mpCamera;

    cv::Ptr<cv::ORB> orb_;

    std::vector<cv::Point3f> pts_3d_ref;
    std::vector<cv::Point3f> pts_3d_all;


    std::vector<cv::KeyPoint> key_points_curr_;

    cv::Mat desc_curr_, desc_ref_;

    std::vector<cv::DMatch> feature_matches_;

    cv::Mat T_c_r_estimate_;

    int frameid;

    int num_inliers_;

    int num_lost_;

    // orb parameter

    int num_of_fts;
    double scale_factor;
    int level_pyramid;
    float match_ratio;
    int max_num_lost;
    int min_inliers;

    // key frame min rot and trans
    double key_frame_min_rot;
    double key_frame_min_trans;

  public:
    VisualOdometry(std::string settingPath, Viewer pViewer);
    ~VisualOdometry();

    cv::Mat Tracking(cv::Mat im, cv::Mat imD, double tFrame);

  private:
    cv::Mat addFrame(Frame* frame);

    void ExtractORB();
    void featureMatching();
    void poseEstimationPnp();
    void setRef3DPts();


    void addKeyFrame();
    bool checkEstimatedPose();
    bool checkKeyFrame();
};

} // namespace slam_fe