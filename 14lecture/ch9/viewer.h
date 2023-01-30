#pragma once
#include <string>

#include "visual_odometry.h"

#include "opencv2/opencv.hpp"
#include "pangolin/pangolin.h"

namespace slam_fe {
class VisualOdometry;
class Viewer {
    public:
    Viewer(std::string settingPath);

    void Run();

    void setCurrentCameraPose(const cv::Mat& Tcw);    

    void setVisualOdometry(VisualOdometry* vo);

    void GetAllFrame(std::vector<Frame*> allFrame);

    void getAll3dPoints(std::vector<cv::Point3f> all3dPts);

    std::mutex mtx_;

    private:
    void DrawCurrentCamera(pangolin::OpenGlMatrix* matrix);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix* M);

    void DrawTrackingFrame();
    void DrawMapPoints();

    cv::Mat DrawFrame();


    pangolin::View d_cam;
    pangolin::OpenGlMatrix Tcw;

    pangolin::OpenGlRenderState s_cam;

    std::mutex mutexCamera;
    std::mutex mutexTrackingFrame;
    std::mutex mutex3dPoints;
    std::mutex mutex;

    std::vector<cv::Point3f> all3dPts;

    std::vector<cv::KeyPoint> kps;

    std::vector<cv::DMatch> feature_matches;
    std::vector<Frame*> allFrames;

    cv::Mat im;
    int n;

    int state;

    double m;
    float imageWidth, imageHeight;

    float viewPointx, viewPointy, viewPointz, ViewPointF;

    float KeyFrameSize;
    float KeyFrameLineWidth;
    float GraphLineWidth;
    float PointSize;
    float CameraSize;
    float CameraLineWidth;
    cv::Mat cameraPose;

    VisualOdometry* vo_;
};
} // namespace of slam_fe