#include "map.h"

namespace slam_fe {

void Map::insertKeyFrame(Frame*  frame) {
    std::cout << "key frame size = " << keyFrames_.size() << std::endl;
    if (keyFrames_.find(frame->id_) == keyFrame_.end()) {
        keyFrame_.insert(std::make_pair(frame->id_,frame));
    } else {
        // update keyFrame.
        keyFrame_[frame->id_] = frame;
    }
}

void Map::insertMapPoint(MapPoint * mp) {
    if (map_points_.find(mp->id_) == map_points_->end()) {
        map_points_.insert(std::make_pair(mp->id_, mp));
    } else {
        // update mapPoint.
        map_points_[mp->id_] = mp;
    }
}

} // namespace slam_fe