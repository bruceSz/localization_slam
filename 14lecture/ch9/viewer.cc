#include "viewer.h"

namespace slam_fe {
Viewer::Viewer(std::string settingPath) {
    cv::FileStorage setting(settingPath, cv::FileStorage::READ);

    float fps = setting["camera.fps"];
    if (fps < 1) {
        fps = 30;
    }

    mT = 1e3/fps;

    imageWidth = setting["camera.width"];
    imageHeight = setting["camera.height"];
    if (imageWidth < 1 && imageHeight < 1) {
        imageWidth = 640;
        imageHeight = 480;
    }

    viewPointx = setting["viewer.viewPointx"];
    viewPointy = setting["viewer.viewPointy"];
    viewPointz = setting["viewer.viewPointz"];

    viewPointF = setting["viewer.viewPointF"];



    mKeyFrameSize = setting["viewer.KeyFrameSize"];
    mKeyFrameLineWidth = setting["viewer.KeyFrameLineWidth"];
    mGraphLineWidth = setting["viewer.GraphLineWidth"];
    mPointSize = setting["viewer.pointSize"];
    mCameraSize = setting["viewer.CameraSize"];
    mcameraLineWidth = setting["viewer.CameraLineWidth"];

    VisualOdometry_ = nullptr;
}

} // namespace slam_fe