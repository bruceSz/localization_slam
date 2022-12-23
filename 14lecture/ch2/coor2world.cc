
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>


using namespace Eigen;
using namespace std;

int main(int argc, char **argv) {
    Quaterniond q_wr(0.55, 0.3, 0.2, 0.2);
    Quaterniond q_rb(0.99, 0,0,0.01);
    Quaterniond q_bl(0.3, 0.5, 0,20.1);
    Quaterniond q_bc(0.8, 0.2, 0.1, 0.1);

    q_wr.normalize();
    q_rb.normalize();
    q_bl.normalize();
    q_bc.normalize();

    Eigen::Vector3d t_wr(0.1, 0.2,0.3);
    Vector3d t_rb(0.05, 0, 0.5);
    Vector3d t_bl(0.4, 0, 0.5);
    Vector3d t_bc(0.5, 0.1, 0.5);

    Vector3d p_c(0.3, 0.2, 1.2);

    Isometry3d T_wr(q_wr);
    T_wr.pretranslate(t_wr);

    Isometry3d T_rb(q_rb);
    T_rb.pretranslate(t_rb);

    Isometry3d T_bl(q_bl);
    T_bl.pretranslate(t_bl);

    Isometry3d T_bc(q_bc);
    T_bc.pretranslate(t_bc);


    Vector3d p_l;
    Vector3d p_w;
    p_l = T_bl.inverse() * T_bc * p_c;
    p_w = T_wr * T_rb * T_bc * p_c;

    cout << "pc coordinates: " << p_c << std::endl;
    cout << "pl coordinates: " << p_l << std::endl;
    cout << "pw coordinates: " << p_w << std::endl;

}