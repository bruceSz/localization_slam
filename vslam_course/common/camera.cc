
#include "camera.h"

#include <opencv2/core/core.hpp>


namespace zs {

Camera::Camera(int id, const std::string& name): id_(id), name_(name),
    fx(0.0), fy(0.0), cx(0.0), cy(0.0 ), k1(0.0), k2(0.0) {}


Camera::~Camera() {}

void Camera::loadC(const std::string& filename) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    if (!fs.isOpened()) {
        return;
    }
    cv::FileNode n = fs["camera"]["intrinsic"];
    fx = static_cast<float>(n["fx"]);
    fy = static_cast<float>(n["fy"]);
    cx = static_cast<float>(n["cx"]);
    cy = static_cast<float>(n["cy"]);
    
    n = fs["camera"]["distortion"];
    k1 = static_cast<float>(n["k1"]);
    k2 = static_cast<float>(n["k2"]);
}

}
