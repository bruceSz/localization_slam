#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <thread>
#include <iomanip>

#include "viewer.h"
#include "visual_odometry.h"

namespace slam_fe {

using namespace std;

class System {
  public:
    
    System(const string& settingPath);

    cv::Mat TrackingRGBD(cv::Mat im, cv::Mat imD, double tFrame);

  public:
    Viewer* mpViewer;
    std::thread* mptViewer;

    VisualOdometry* mVO;

};

} // namespace slam_fe