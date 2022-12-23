#include <iostream>

#include <Eigen/Core>

#include <Eigen/Dense>

using namespace std;

constexpr int  MATRIX_SIZE = 10;

int main(int argc, char **argv) {
    Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_nn;
    matrix_nn = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    
    Eigen::Matrix<double, MATRIX_SIZE, 1 >  v_nd;
    v_nd = Eigen::MatrixXd::Random(MATRIX_SIZE,1);

    Eigen::Matrix<double, MATRIX_SIZE,1> x_inverse;
    Eigen::Matrix<double, MATRIX_SIZE, 1 > x_qr;

    x_inverse = matrix_nn.inverse() * v_nd;
    x_qr = matrix_nn.colPivHouseholderQr().solve(v_nd);

    cout << "Matrix " << matrix_nn << std::endl;
    cout << "v_nd " << v_nd << std::endl;

    cout << "x_inverse: " << x_inverse << std::endl;

    cout << "x_qr: " << x_qr << std::endl;

}