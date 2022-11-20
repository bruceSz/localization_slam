#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/se3.hpp"

using namespace std;
using namespace Eigen;

/// 本程序演示sophus的基本用法


int main(int argc, char** argv) {


  cout << "half of pi: " << M_PI_2/M_PI*180 << std::endl;
  Matrix3d R = AngleAxisd(M_PI_2, Vector3d(0,0,1)).toRotationMatrix();
  cout << "-------------------------------- rotation matrix:\n " << R << endl;

  Quaterniond q(R);

  Sophus::SO3d so3_r(R);
  Sophus::SO3d so3_q(q);
  cout << "SO(3) from matrix: \n" << so3_r.matrix() << endl;
  cout << "SO(3) from quaternion: \n" << so3_q.matrix() << endl;

  Vector3d so3 = so3_r.log();
  cout << "SO(3) log: \n" << so3  << " norm of so3 " << so3.norm()<< endl;
  cout << "so3 hat = \n" << Sophus::SO3d::hat(so3) << endl;
  cout << "so3 hat vee=\n" << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)) << endl;

  Vector3d update_so3(1e-3,0,0);
  Sophus::SO3d SO3_update = Sophus::SO3d::exp(update_so3) * so3_r;
  // left multiplication
  cout << "SO3 updated:\n" << SO3_update.matrix() << std::endl;


  Vector3d t(1,0,0);
  Sophus::SE3d se3_rt(R,t);
  Sophus::SE3d se3_qt(q,t);

  cout << "SE3 from R,t:\n" << se3_rt.matrix() << endl;
  cout << "SE3 from q,t:\n"<< se3_qt.matrix() << endl;

  // lie  se3 is 6 degree vector
  typedef Eigen::Matrix<double, 6,1> Vector6d;

  Vector6d se3 = se3_rt.log();

  cout << "se3(6d):\n" << se3 << endl;
  cout << "se3(6d) norm:\n" << se3.norm() << endl;

  cout << "se3(6d): hat:\n" << Sophus::SE3d::hat(se3) << endl;
  cout << "se3(6d): vee = \n"<< Sophus::SE3d::vee(Sophus::SE3d::hat(se3)).transpose() << endl;


}


