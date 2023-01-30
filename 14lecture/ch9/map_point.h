#pragma once

namespace slam_fe {

class Frame;
class MapPoint {
    public:
    typedef  shared_ptr<MapPoint> Ptr;
    unsigned long id_;
    cv::Vector3d pos_; // position in the world
    cv::Vector3d norm_; // normal in view direction
    cv::Mat      descriptor_; // descriptor for matching.
    int observed_times_; // being observed by feature matching algo.
    int corrected_times_; // being an inlinear in pose  estimation

    MapPoint();
    MapPoint(long id, Vector3d pos, Vector3d norm);

    static MapPoint::Ptr createMapPoint();
};


}