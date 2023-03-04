#include "dlt.h"

#include <iostream>
#include <cassert>
#include "common/pose.h"

namespace zs {
DLT::DLT(const Eigen::Matrix<double, Eigen::Dynamic, 3>& world_pts,
        const Eigen::Matrix<double, Eigen::Dynamic,2>& pxs) {
    world_pts_ =  world_pts;
    pxs_ = pxs;
    camera_k_ = Eigen::Matrix3d::Zero();
    assert(world_pts.rows() == pxs.rows());
    pt_num_  = world_pts.rows();
}


DLT::DLT(const Eigen::Matrix<double, Eigen::Dynamic, 3>& world_pts,
        const Eigen::Matrix<double, Eigen::Dynamic, 2>& pxs,
        const Eigen::Matrix3d& cam_k ) {
    world_pts_ = world_pts;
    pxs_ = pxs;
    camera_k_ = cam_k;
    assert(world_pts.rows()== pxs.rows());
    pt_num_ = world_pts.rows();
}

// ref : https://blog.csdn.net/rs_lys/article/details/117594660 dla to compute k and R and t.
// ref : https://blog.csdn.net/rs_lys/article/details/116332989 zhang' style calibration.
// ref : https://blog.csdn.net/mucai1/article/details/85242098 matrix decomposition
// ref : https://blog.csdn.net/weixin_42587961/article/details/97241162 why ax=0's solution is v's last col: 
// ref : https://blog.csdn.net/weixin_42587961/article/details/97374248 opencv & svd
void DLT::run() {
    Eigen::Matrix<double, Eigen::Dynamic, 12> Q = makeQ();
    Eigen::Matrix<double, 3, 4> M = getM(Q);

    if(camera_k_ == Eigen::Matrix3d::Zero()) {
        decompM(M, pose_, camera_k_);
    } else {
        pose_ = poseFromM(M);
    }

}

// reference: https://zhuanlan.zhihu.com/p/58648937
Pose3D DLT::poseFromM(const Eigen::Matrix<double, 3, 4> M) {
    Eigen::Matrix3d R = M.block(0,0,3,3);
    Eigen::JacobiSVD<Eigen:::MatrixXd> svd(R, Eigen::ComputeThinU|Eigen::ComputeThinV);
    Eigen::MatrixXd U, S, V;
    U = svd.matrixU();
    V = svd.matrixV();
    S = svd.singularValues();

    double d0 = R.norm();
    R = U * V.transpose();

    double alpha = R.norm() / d0;
    Eigen::Vector3d t = M.block(0,3,3,1);
    t *= alpha;

    return Pose3D(R, t);
}


void DLT::decompM(const Eigen::Matrix<double, 3, 4> M, Pose3D& pose, Eigen::Matrix3d& K) {
    Eigen::Matrix3d KR = M.block(0,0,3,3);
    Eigen::HouseholderQR<Eigen::MatrixXd> qr;
    qr.compute(KR);
    Eigen::MatrixXd R = qr.matrixQR().triangularView<Eigen::Upper>();
    Eigen::MatrixXd Q = qr.householderQ();

    K = R;
    K = K/ K(2,2);

    // 180 rotation
    Eigen::Matrix3d r_180;
    r_180 << -1, 0 , 0,
            0, -1, 0,
            0, 0, 1;
    //TODO: test this rotation. 
    if (K(0,0) < 0) {
        //? k = k* r
        //? q = r * q;
        K = K * r_180;
        Q = r_180 * Q;
    }



    Eigen::Vector3d Kt = M.block(0,3,3,1);
    Eigen::Vector3d t = K.inverse() * Kt;
    pose.R = Q;
    pose.t = t;
}

Eigen::Matrix<double,3,4> DLT::getM(const Eigen::Matrix<double, Eigen::Dynamic, 12>& Q) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Q,Eigen::ComputeThinU|Eigen::ComputeThinV);
    Eigen::MatrixXd U, S, V;
    U = svd.matrixU();
    V = svd.matrixV();
    S = svd.singularValues();
    //reference: https://blog.csdn.net/weixin_42587961/article/details/97374248

    Eigen::VectorXd solu = V.col(V.cols() - 1);
    Eigen::Matrix<double, 3, 4> M;
    M << solu(0), solu(1), solu(2), solu(3),
        solu(4), solu(5), solu(6), solu(7),
        solu(8), solu(9), solu(10), solu(11);

    if(M.block(0,0, 3,3).determinant() < 0) {
        M *= -1;
    }
    return M;
}

Eigen::Matrix<double, Eigen::Dynamic, 12> DLT::makeQ() {
    Eigen::Matrix<double, Eigen::Dynamic, 12> Q;
    Q.resize(pt_num_ * 2, 12);
    if(camera_k_ == Eigen::Matrix3d::Zero()) {
        // unknown intrinsic matrix.
        for(int i=0; i< pt_num_; i++) {
            Eigen::Matrix<double, 12, 1> row;
            row << world_pts_(i,0) , world_pts_(i,1), world_pts_(i,2), 1,
                 0, 0, 0, 0, 
                 -pxs_(i, 0) * world_pts_(i, 0), -pxs_(i, 0) * world_pts_(i, 1),
                 -pxs_(i, 0) * world_pts_(i, 2), -pxs_(i, 0);
            Q.row(2*i)  = row;
            row << 0, 0 , 0, 0,
                    world_pts_(i,0), world_pts_(i,1), world_pts_(i,2), 1,
                    -pxs_(i, 1) * world_pts_(i, 0), -pxs_(i, 1) * world_pts_(i, 1),
                    -pxs_(i, 1) * world_pts_(i, 2), -pxs_(i, 1);
            Q.row(2*i + 1) = row;

        }
        
    } else {
        for(int i=0; i< pt_num_; i++) {
            // normalized plane.
            Eigen::Vector3d inverse_k_px = camera_k_.inverse() * Eigen::Vector3d(pxs(i,0), pxs(i,1), 1);
            Eigen::Matrix<double, 12, 1> row;

            row << world_pts_(i,0) , world_pts_(i,1), world_pts_(i,2), 1,
                 0, 0, 0, 0, 
                 -inverse_k_px(0) * world_pts_(i, 0), -inverse_k_px(0) * world_pts_(i, 1),
                 -inverse_k_px(0) * world_pts_(i, 2), -pxs_(i, 0);

            Q.row(2*i)  = row;
            row << 0, 0 , 0, 0,
                    world_pts_(i,0), world_pts_(i,1), world_pts_(i,2), 1,
                    -inverse_k_px(1) * world_pts_(i, 0), -inverse_k_px(1) * world_pts_(i, 1),
                    -inverse_k_px(1) * world_pts_(i, 2), -inverse_k_px(1);
            Q.row(2*i + 1) = row;
        }

    }
    return Q;
}


} // namespace zs