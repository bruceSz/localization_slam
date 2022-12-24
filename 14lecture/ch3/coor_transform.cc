#include <iostream>
#include <vector>
#include <algorithm>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

using namespace std;
using namespace Eigen;


int main(int argc, char** argv) {

  Quaterniond q1(0.35, 0.2, 0.3, 0.1);
  Quaterniond q2(-0.5, 0.4, -0.1, 0.2);

  cout << "q1: " << q1.coeffs() << endl;
  cout << "q2: " <<  q2.coeffs() << endl;


  q1.normalize();
  q2.normalize();
  cout << "after normalization" << endl;

  cout << "q1 normalize: " <<  q1.coeffs() << endl;
  cout << "q2 normalize: " <<  q2.coeffs() << endl;
}

int main_deprecated(int argc, char** argv) {
  Quaterniond q1(0.35, 0.2, 0.3, 0.1), q2(-0.5, 0.4, -0.1, 0.2);
  q1.normalize();
  q2.normalize();
  Vector3d t1(0.3, 0.1, 0.1), t2(-0.1, 0.5, 0.3);
  Vector3d p1(0.5, 0, 0.2);

  Isometry3d T1w(q1), T2w(q2);
  T1w.pretranslate(t1);
  T2w.pretranslate(t2);

  Vector3d p2 = T2w * T1w.inverse() * p1;
  cout << endl << p2.transpose() << endl;
  return 0;
}