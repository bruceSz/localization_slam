
#include <iostream>
#include <memory>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/optimization_algorithm_levenberg.h>



class hello_vertex : public g2o::BaseVertex<1, Eigen::Matrix<double,1,1>> {

    virtual void oplusImpl(const double* v);

    virtual void setToOriginImpl(); 

    virtual bool read(std::istream& is) {
        return true;
    }
    virtual bool write(std::ostream& os) const {
        return true;
    }
};


class hello_edge: public g2o::BaseUnaryEdge<1, double, hello_vertex> {
    virtual void computeError() override;
    virtual void linearizeOplus() override ;


    virtual bool read(std::istream& is ) {
        return true;
    }

    virtual bool write(std::ostream& os) const {
        return true;
    }
};



// en is en
// landmarkdim is vn
template<int en, int vn, class EDGE, class VERTEX>
void solve(Eigen::Matrix<double, 1, vn>& estimate_init,
        Eigen::Matrix<double,1, en>& measurement_init) {
// edge: pose, vertex: landmark
  using Block =  g2o::BlockSolver<g2o::BlockSolverTraits< en, vn>>;
  
  
    std::unique_ptr<typename Block::LinearSolverType>  
     linearSolver(new g2o::LinearSolverDense<typename Block::PoseMatrixType>());

    std::unique_ptr<Block> solver_ptr(new Block(std::move(linearSolver)));

    g2o::OptimizationAlgorithmLevenberg * solver =
        new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));


    // why this?
    // graph model.
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    optimizer.setVerbose(true);

    // init and  add vertex.
    VERTEX* v = new VERTEX();
    //Eigen::Matrix<double,1, vn> estimate_init;
    //estimate_init
    // this value will be overwriten by setToOriginImpl
    v->setEstimate(estimate_init);
    v->setId(0);

    optimizer.addVertex(v);
    EDGE* edge  = new EDGE();
    
    edge->setId(0);
    edge->setVertex(0,v);
    edge->setMeasurement(measurement_init);
    edge->setInformation(Eigen::Matrix<double,en,vn>::Identity() * 1/ 0.25);
    
    optimizer.addEdge(edge);

    optimizer.initializeOptimization();
    optimizer.optimize(100);
    Eigen::Matrix<double, 1, vn> est_estimate = v->estimate();
    std::cout << "Estimated value: " << est_estimate << std::endl;

    
    return ;

}