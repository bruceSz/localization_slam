#pragma once

#include <eigen3/Eigen/Core>

namespace zs {

template<class PoseDType>
class _Pose3D {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    _Pose3D( const PoseDType& wx, 
            const PoseDType& wy, 
            const PoseDType& wz,
            const PoseDType& tx, 
            const PoseDType& ty,
            const PoseDType& tz) : wx(wx), wy(wy), wz(wz), tx(tx), ty(ty), tz(tz) {}

    ~_Pose3D() {}

    _Pose3D& operator=(const _Pose3D& rhs) {
        wx = rhs.wx; wy = rhs.wy; wz = rhs.wz;
        tx = rhs.tx; ty = rhs.ty; wz = rhs.tz;
        return *this;
    }

    _Pose3D(const _Pose3D& rhs) {
        wx = rhs.wx; wy = rhs.wy; wz = rhs.wz;
        tx = rhs.tx; ty = rhs.ty; wz = rhs.tz;
        
    }

    Eigen::Matrix<double, 3, 3> R() const {
        // rodrigues
        PoseDType theta = sqrtl(wx * wx + wy * wy + wz * wz);
        Eigen::Matrix<PoseDType, 3,3> _R, k;
        // negative sysmetric matrix
        k << 0, -wz/theta, wy/theta, 
             wz/theta, 0, -wx/theta, 
             -wy/theta, wx/theta, 0;

        _R = cos(theta)*Eigen::Matrix<PoseDType, 3, 3>::Identity()+ (1-cos(theta)) * k * k + sin(theta) * k;

        return _R;
    }

    Eigen::Matrix<PoseDType, 3, 1> t() const {
        return Eigen::Matrix<PoseDType, 3, 1>(tx, ty, tz);
    }



    Eigen::Matrix<PoseDType, 4,4> Rt() const {
        Eigen::Matrix<PoseDType, 4,4> _T = Eigen::Matrix<PoseDType, 4, 4>::Zero();
        _T.block(0,0, 3,3) = R();
        _T.block(0,3, 3,1) = t();
        return _T;

    }

private:
    PoseDType wx, wy, wz, tx, ty, tz;
};

using Pose3D  = _Pose3D<double>;

}