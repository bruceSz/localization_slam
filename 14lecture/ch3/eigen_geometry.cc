#include <iostream>
#include <cmath>

using namespace std;

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

using namespace Eigen;

// 本程序演示了 Eigen 几何模块的使用方法


int main(int argc, char **argv) {
  Matrix3d rotation_matrix = Matrix3d::Identity();

    // arround z axis, rotate counter-clockwise 
  AngleAxisd rotation_vector(M_PI/4, Vector3d(0,0,1));
  cout.precision(10);

  cout << "rotation_matrix from angleAxis=\n" << rotation_vector.matrix() << endl;


  rotation_matrix = rotation_vector.toRotationMatrix();

  cout << "rotation matrix from assignment =\n" << rotation_matrix << std::endl;

  cout << "coor transform using angleAxis" << std::endl;

  Vector3d v(1,0,0);
  Vector3d v_r = rotation_vector * v;
  cout << "(1,0,0) after rotation (by angle axis(pi/4,[0,0,1])) = " << v_r.transpose() << endl;

  v_r = rotation_matrix * v;
  cout << "(1,0,0) after rotation rotation_matrix:\n" << v_r<< std::endl;


  // ZYX order
  Vector3d euler_ang = rotation_matrix.eulerAngles(2,1,0);
  cout << "euler_ang =\n" << euler_ang * 180/M_PI << "#"<< euler_ang << std::endl;

    cout << "xxxx" << std::endl;
  Isometry3d T = Isometry3d::Identity();
  T.rotate(rotation_vector);
  T.pretranslate(Vector3d(1,3,4));
  cout << "T =\n" << T.matrix() << std::endl;

  Vector3d v_T = T * v;
  cout << "v_T =\n" << v_T << std::endl;

  Quaterniond q(rotation_vector);
  cout << "q from rotation vector=\n" << q.coeffs() 
  //<< "\n\n rotation vector: " << rotation_vector << std::endl
    << std::endl;


    q = rotation_matrix;
    cout << "q from rotation matrix=\n" << q.coeffs() << std::endl;

    Vector3d v_q = q * v;
    cout << "v_q =\n" << v_q << std::endl;

    // w then v
    cout << "should be equal to " << (q * Quaterniond(0, 1, 0, 0) * q.inverse()).coeffs().transpose() << endl;
}
