#pragma once

#include "common/pose.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Cholesky>
// L * L^*

namespace zs {

class DLT {
  public:
    DLT(const Eigen::Matrix<double, Eigen::Dynamic, 3>& world_pts, 
        const Eigen::Matrix<double, Eigen::Dynamic, 2>& pxs);

    DLT(const Eigen::Matrix<double, Eigen::Dynamic,3> world_pts, 
        const Eigen::Matrix<double, Eigen::Dynamic,2>& pxs, 
        const Eigen::Matrix3d& cam_k);

    ~DLT() = default;

    void run();


    Pose3D get_pose() {
        return pose_;
    }

    Eigen::Matrix3d get_cam_k() {
        return cam_k;
    }


    private:
    Eigen::Matrix<double, Eigen::Dynamic, 12> makeQ();
    Eigen::Matrix<double,3,4> getM(const Eigen::Matrix<double, Eigen::Dynamic, 12>& Q);
    void decompM(const Eigen::Matrix<double, 3, 4> M, Pose3D& pose, Eigen::Matrix3d& K);

    Pose3D poseFromM(const Eigen::Matrix<double, 3, 4> M);

    Eigen::Matrix<double, Eigen::Dynamic, 3> world_pts_;
    Eigen::Matrix<double, Eigen::Dynamic, 2> pxs_;
    Eigen::Matrix3d camera_k_;
    Pose3D pose_;
    int pt_num_;
         
};

} // namespace zs