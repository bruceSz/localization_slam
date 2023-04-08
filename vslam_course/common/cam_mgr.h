

#pragma once

#include "camera.h"
#include <map>


namespace zs {
class CameraMgr {

public:
    static CameraMgr& GetInstance() { 
        static CameraMgr ins;
        return ins;
    };


    ~CameraMgr();

    void addCamera(Camera::CameraPtr& cam);

    Camera::CameraPtr getCameraById(int id)  ;

private:
    CameraMgr();
    CameraMgr(const CameraMgr& rhs);

    CameraMgr& operator=(const CameraMgr& rhs);

    std::map<int, Camera::CameraPtr> camera_ids_;

};
} // namespace zs