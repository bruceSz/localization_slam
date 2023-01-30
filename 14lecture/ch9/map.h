#pragma once

namespace slam_fe {
class Map{
    public:
    typedef std::shared_ptr<Map> Ptr;
    unordered_map<unsigned long, MapPoint::Ptr> map_points_; 
    unordered_map<unsigned long, Frame::Ptr> keyFrames_;

    Map() {}

    void insertKeyFrame(Frame::Ptr frame);
    void insertMapPoint(MapPoint::Ptr map_point);
};
}