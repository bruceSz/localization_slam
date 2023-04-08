#include "cam_mgr.h"

namespace zs {

CameraMgr::CameraMgr() {}

CameraMgr::~CameraMgr() {}

CameraMgr::CameraMgr(const CameraMgr& other) {}

CameraMgr& CameraMgr::operator=(const CameraMgr& other) {
    return *this;
}

void CameraMgr::addCamera(Camera::CameraPtr& cam) {
    if (camera_ids_.find(cam->id()) != camera_ids_.end()) {
        camera_ids_[cam->id()] = cam;
    } else {
        camera_ids_.insert(std::pair<int, Camera::CameraPtr>(cam->id(), cam));
    }
}

Camera::CameraPtr CameraMgr::getCameraById(int id)  {
    if (camera_ids_.find(id) != camera_ids_.end()) {
        return camera_ids_[id];
    }
    return nullptr;
}

}