#include <iostream>

//#include <g2o/core/g2o_core_api.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include <Eigen/Core>
//#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

#include "g2o_common.h"

using std::endl;
using std::cout;


class power_vertex: public g2o::BaseVertex<4, Eigen::Matrix<double, 1, 4>> {
    virtual void oplusImpl(const double *v) {
        cout << "oplus: " << endl;
        cout << Eigen::Matrix<double, 1,4>(v) << endl;
        _estimate += Eigen::Matrix<double, 1,4>(v);
        cout << "_estimate: " << _estimate << endl;
    }

    virtual void setToOriginImpl() {
        cout << "init _estimate."  << endl;
        _estimate << 0,0,0,0;
    }

    virtual bool read(std::istream& is) {
        return true;
    }
    virtual bool write(std::ostream& os) const override {
        return true;
    }

};

class power_edge : public g2o::BaseUnaryEdge<4, Eigen::Matrix<double, 1,4>, power_vertex> {
    virtual void computeError () {
        const power_vertex* v = static_cast<const power_vertex*>(_vertices[0]);
        Eigen::Matrix<double, 1,4> est = v->estimate();
        
        /*
         
         f1(x) =x1+10x2
         f2(x)=5–√(x3−x4)
         f3(x)=(x2−2x3)^2
         f4(x)=10−−√(x1−x4)^2

         F(x) =[f1(x), f2(x), f3(x), f4(x)]
        */
        _error(0,0) = _measurement(0,0) - (est(0,0) + 10*est(0,1));
        _error(1,0) = _measurement(0,1) - sqrt(5)* (est(0,2) - est(0,3));
        _error(2,0) = _measurement(0,2) - (est(0,1) - 2*est(0,2) * (est(0,1) - 2*est(0,2)));
        _error(3,0) = _measurement(0,3) - sqrt(10) * (est(0,0)-est(0,2)) * (est(0,0) - est(0,2));
        cout << "Error: " << _error.transpose() << endl;
    }

    virtual bool read(std::istream& is) {
        return true;
    }
    virtual bool write(std::ostream& os) const override{
        return true;
    }
};

int main(int argc, char ** argv) {
    Eigen::Matrix<double,1,4> estimate_initial;
    estimate_initial <<0.5,1.5,2.5,3.5;

    Eigen::Matrix<double,1,4> measurement_initial;
    measurement_initial << 21,-sqrt(5),16,9*sqrt(10);

    solve<4,4, power_edge, power_vertex>(estimate_initial, measurement_initial);
    return 1;
}