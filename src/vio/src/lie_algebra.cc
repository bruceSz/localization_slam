
#include <iostream>
#include <Eigen/Dense>

using namespace  std;
using namespace Eigen;




Quaterniond rotation_matrix_update(Matrix3d r_matrix) {
     // a simple rotation axis and theta. with theta and unit vector.
     
     
     // target lie algebra/ rotation vector.
     // according to definition:
     //  theta:            w is norm of w 
     //  n(rotation axis): is the unit vector computed from w
     Vector3d w(0.01, 0.02, 0.03) ;
     
     //std::cout << " old: rotation matrix" << "\n"
     //   << r_matrix << std::endl;
     // same with norm.
     //auto w_length = sqrt(w[0]*w[0]+w[1]*w[1]+w[2]*w[2]);
     
     
    auto theta = w.norm();
    
    auto n = w;
    n.normalize();
    // n is unit vector now !
    Matrix3d os_matrix  ;
    os_matrix(0,0) = 0;
    os_matrix(0,1) = -n[2];
    os_matrix(0,2) = n[1];
    os_matrix(1,0) = n[2];
    os_matrix(1,1) = 0;
    os_matrix(1,2) = -n[0];
    os_matrix(2,0) = -n[1];
    os_matrix(2,1) = n[0];
    os_matrix(2,2) = 0;

    // rodrigues'formula: cos(theta)*I + (1-cos(theta))*n*n^T + sin(theta)n_hat(n's oppsite-symmetric matrix: os_matrix)
    auto R = cos(theta) * Matrix3d::Identity() + (1-cos(theta))*n*n.transpose() + sin(theta)*os_matrix;
    auto ret = r_matrix * R;
    //std::cout << "Matrix after update:" << "\n"
    //    << ret << std::endl;
    Quaterniond q;
    q = ret;
    std::cout << "update with R , ret quaternion: " << "\n" 
        << q.coeffs() << std::endl;
    return q;
      
}

Quaterniond rotation_matrix_update_with_q(Matrix3d r_matrix ) {
    Vector3d w(0.01, 0.02, 0.03) ;
    auto theta = w.norm();
    
    auto n = w;
    n.normalize();

    AngleAxisd t_v(theta,n);

    Quaterniond q_update(t_v);
    
    //std::cout << "quaternion of a rotaiton vec:" << "\n"
    //    << "x: " << q_update.x()  << "\n"
    //    << "y: " << q_update.y()  << "\n"
    //    << "z: " << q_update.z()  << "\n"
    //    << "w: " << q_update.w()  << "\n"
    //    << std::endl;

    //q = (1, 1/2*w)
    Quaterniond q_old;
    q_old = r_matrix;

    auto ret = q_old* q_update;
    std::cout << "update with q, result quaternion: " << "\n" 
        << ret.coeffs() << std::endl;

    
}


int main(int argc, char**argv) {
    std::cout << "hello" << std::endl;
    AngleAxisd t_V(M_PI / 4, Vector3d(0, 0, 1));
    auto r_matrix = t_V.toRotationMatrix();
    rotation_matrix_update(r_matrix);
    rotation_matrix_update_with_q(r_matrix);
}