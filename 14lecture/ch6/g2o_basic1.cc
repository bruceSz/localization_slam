
#include <iostream>

//#include <g2o/core/g2o_core_api.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>



#include <Eigen/Core>
//#include <g2o/core/optimization_algorithm_dogleg.h>


#include "g2o_common.h"

using std::cout;
using std::endl;



void hello_vertex::oplusImpl(const double* v)  {
    cout << "v: " << *v << endl;
    _estimate(0,0) += *v;
}


void hello_vertex::setToOriginImpl(){
        cout << "setToOriginImpl: " << _estimate(0,0) << endl;
        _estimate(0,0) = 0;
}


void hello_edge::computeError()  {
    const hello_vertex* v = static_cast<const hello_vertex*> (_vertices[0]);

    const Eigen::Matrix<double, 1, 1> abc = v->estimate();
    cout << "computeError:" ;
    _error(0,0) = -_measurement + (abc(0,0) + 1) * (abc(0,0) + 1);
    cout << _error(0,0) << endl;

}

void hello_edge::linearizeOplus()  {
    const hello_vertex * v = static_cast<const hello_vertex*>(_vertices[0]);
    const Eigen::Matrix<double, 1,1> abc = v->estimate();


    _jacobianOplusXi(0,0) = 2*(abc(0,0) + 1);

    cout << "_jacobianOplusXi" << _jacobianOplusXi << endl;
}


int main(int argc, char** argv) {
    
    
    return 0;
  
}