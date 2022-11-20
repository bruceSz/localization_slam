

#include <vector>
#include <iostream>
#include <chrono>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>

using std::vector;
using std::endl;
using std::cout;
using namespace std::chrono;


struct CURVE_FITTING_COST {
    CURVE_FITTING_COST(double x, double y) : _x(x), _y(y) {}

    template<typename T>
    bool operator() (
        const T * const abc,
        T* res) const {
            res[0] = T(_y) - ceres::exp(abc[0]*T(_x)*T(_x) + abc[1]*T(_x) + abc[2]);
            return true;
        }
    

    const double _x, _y;
};

int main(int argc, char** argv) {
    double ar = 1.0, br = 2.0, cr = 1.0;
    double ae = 1.0, be = -1.0, ce = 5.0;

    int N = 100;

    double w_sigma = 1.0;
    //double inv_sigma = 1.0/ w_sigma;

    cv::RNG rng;

    vector<double> xD,yD;

    for(int i = 0; i < N; i++) {
        double x = i/100;
        xD.push_back(x);
        yD.push_back(exp(ar*x*x + br * x + cr) + rng.gaussian(w_sigma*w_sigma));

    }
    double abc[3] = {ae, be, ce};


    ceres::Problem problem;
    for(int i = 0; i <N; i++) {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST,1,3>(
                new CURVE_FITTING_COST(xD[i],yD[i])
            ),
            // kernel function, not used here, hence nullptr
            nullptr,
            abc
        );
    }

    ceres::Solver::Options options;
    //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    ceres::Solve(options, &problem, &summary);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
    cout << "solve time cost: " << time_used.count() << endl;

    cout << summary.BriefReport() << endl;
    cout << "estimated abc: " ;
    for (auto i: abc) {
        cout << i << " " ;
    }
    cout << endl;


    return 1;

}