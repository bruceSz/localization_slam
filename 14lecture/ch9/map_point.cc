#include "map_point.h"

namespace slam_fe {

MapPoint::MapPoint() : id_(-1), pos_(cv::Point3d(0.0,0.0,0.0)),
    observed_times_(0), corrected_times_(0) {

    }
MapPoint::MapPoint(long id, cv::Point3d pos, cv::Point3d norm): 
    id_(id), pos_(pos),norm_(norm),observed_times_(0), corrected_times_(0) {
    
}

MapPoint* MapPoint::createMapPoint() {
    static long factory_id = 0;

    return (new MapPoint(factory_id, cv::Point3d(0.0,0.0,0.0),
        cv::Point3d(0.0, 0.0, 0.0)));
}

} // namespace slam_fe