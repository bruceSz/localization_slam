#include <iostream>

using namespace std;

#include <ctime>
// Eigen 核心部分
#include <eigen3/Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <eigen3/Eigen/Dense>

using namespace Eigen;

#define MATRIX_SIZE 50

/****************************
* 本程序演示了 Eigen 基本类型的使用
****************************/



int main(int argc, char**) {

     Matrix<float, 2,3> matrix_23;
     matrix_23 << 1,2,3,
                  4,5,6;

     Vector3d v_3d;
     Matrix<float, 3,1> vd_3d;

     Matrix3d matrix_33 = Matrix3d::Zero();

     Matrix<double, Dynamic, Dynamic> matrix_dyn;
     MatrixXd matrix_x;

     cout << "matrix_23: " << matrix_23 << endl;

     cout << "acces by idx" << endl;
     // access matrix
     for(int i=0;i<matrix_23.rows() ;i++) {
          for(int j=0;j<matrix_23.cols(); j++) {
               cout << matrix_23(i,j) << " ";
          }
          cout << endl;
     }


     v_3d << 3,2,1;
     vd_3d << 4,5,6;

     cout << "matrix_23 * v_3d:\n" << matrix_23.cast<double>() * v_3d << endl;
     cout << "matrix_23 * vd_3d:\n" << matrix_23 * vd_3d << endl;


     //TODO finish this code .
}