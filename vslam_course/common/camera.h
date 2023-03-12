#pragma once


#include <string>
#include <memory>

#include <iostream>

namespace zs {

class Camera {
    public:
    Camera(int id, const std::string& name);
    ~Camera();

    void loadC(const std::string& filename) ;


    friend std::ostream& operator<<(std::ostream& os, const  std::shared_ptr<Camera> cameraPtr) {
        os << cameraPtr -> name() 
            << ", fx = " << cameraPtr -> fx
            << ", fy = " << cameraPtr -> fy
            << ", cx = " << cameraPtr -> cx
            << ", cy = " << cameraPtr -> cy
            << ", k1 = " << cameraPtr -> k1
            << ", k2 = " << cameraPtr -> k2;

        return os;
    }

    int id() const {
        return id_;
    }

    std::string name() {
        return name_;
    }

    double fx, fy, cx, cy, k1, k2;

  private:
    int id_;
    std::string name_;
public:
    using CameraPtr  = std::shared_ptr<Camera> ;
};

}