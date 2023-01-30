#include "system.h"

namespace slam_fe {
    System::System(const std::string& settingPath) {
        mpViewer = new Viewer(settingPath);
        mptViewer = new thread(&Viewer::Run, mpViewer);

        mVO = new slam_fe::VisualOdometry(settingPath, mpViewer);
    }

    cv::Mat System::TrackingRGBD(cv::Mat im, cv::Mat imD, double tFrame) {
        return mVO->Tracking(im, imD, tFrame);
    }
};