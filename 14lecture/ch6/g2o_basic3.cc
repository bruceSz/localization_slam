
#include <iostream>
#include <vector>
#include <random>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>


#include <g2o/core/block_solver.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/optimization_algorithm_levenberg.h>


//class g2o_vertex: public BaseVertex<3, Eigen::Matrix<double, 1, 3>> {
class CurveFittingVertex: public g2o::BaseVertex<3, Eigen::Matrix<double, 1,3>> {
    void oplusImpl(const double* v) {
        _estimate += Eigen::Matrix<double, 1,3>(v);
    }

    void setToOriginImpl() {
        _estimate.setZero();
    }

    virtual bool read(std::istream& is) {
        return true;
    }

    virtual bool write(std::ostream& os) const {
        return true;
    }
};

class g2o_curl_fit_edge: public g2o::BaseUnaryEdge<1, double, CurveFittingVertex> {

    public: 
    explicit g2o_curl_fit_edge(double x): _x(x) {}
    void computeError() override {
        CurveFittingVertex * v = static_cast<CurveFittingVertex*>(_vertices[0]);
        const Eigen::Matrix<double, 1, 3> est = v->estimate();
        _error(0,0) = _measurement - exp(est(0,0)*_x*_x + est(0,1)*_x + est(0,2));
    }

    bool read(std::istream& is) { return true;}

    bool write(std::ostream& os) const { return true;}

    private:
    double _x;
};

int main(int argc, char ** argv) {

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<3,1>> block_solver;

    std::unique_ptr<block_solver::LinearSolverType> linearSolver(new g2o::LinearSolverDense<block_solver::PoseMatrixType>);
    std::unique_ptr<block_solver> blk_sol( new block_solver(std::move(linearSolver)));

    g2o::OptimizationAlgorithmLevenberg * algo = new g2o::OptimizationAlgorithmLevenberg(std::move(blk_sol));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(algo);
    optimizer.setVerbose(true);

    CurveFittingVertex* vertex = new CurveFittingVertex();

    Eigen::Matrix<double, 1,3> estimate_initial;
    estimate_initial << 0,0,0;

    vertex->setEstimate(estimate_initial);
    vertex->setId(0);
    optimizer.addVertex(vertex);



    std::vector<double> _x, _y;

    double _tmp_x;
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0, 0.5);

    // avoid too large x, with exp function, it will overflow for big x.
    for (int i = 0; i < 100; i++) {
        _tmp_x = i * 0.005;
        _x.push_back(_tmp_x);
        _y.push_back(exp(3*_tmp_x* _tmp_x + 2*_tmp_x + 1) + distribution(generator));
    }

    for (int i = 0; i < 100; i++) {
        g2o_curl_fit_edge * e = new g2o_curl_fit_edge(_x[i]);
        e->setId(i);
        e->setVertex(0,vertex);
        e->setInformation(Eigen::Matrix<double, 1,1>(1/0.25));
        e->setMeasurement(_y[i]);
        optimizer.addEdge(e);
    }

    optimizer.initializeOptimization();
    optimizer.optimize(100);
    std::cout << "optimized value: " << vertex->estimate().transpose() << std::endl;
    return 0;

}