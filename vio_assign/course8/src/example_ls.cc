#include <iostream>
#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Core>

using Eigen::VectorXd;
using Eigen::MatrixXd;

using std::cerr;
using std::endl;

struct Param {
    int A = 5;
    int B = 1;
    int C = 10;
    int D = 2;
};

#define DERIVE_STEP 0.05
#define MAX_ITER  100

double func(const VectorXd& input, const VectorXd& output, const VectorXd& params, double objIdx) {
    double x1 = params(0);
    double x2 = params(1);
    double x3 = params(2);
    double x4 = params(3);
    
    double t = input(objIdx);
    double f = output(objIdx);
    return x1*sin(x2*t) + x3* cos(x4*t) - f;

}


VectorXd objF(const VectorXd& input, const VectorXd& output, const VectorXd& params) {
    VectorXd obj(input.rows());
    for(int i=0;i< input.rows(); i++) {
        obj(i) = func(input, output, params, i);
    }
    return obj;
}

double sqerror(const VectorXd& obj) {
    return obj.squaredNorm() / 2;
}



double Deriv(const VectorXd& input, const VectorXd& output, int objIdx, const VectorXd& params, int paraIdx) {
        VectorXd para1 = params;
        VectorXd para2 = params;
        para1(paraIdx) += DERIVE_STEP;
        para2(paraIdx) -= DERIVE_STEP;
        double obj1 = func(input, output, para1, objIdx);
        double obj2 = func(input, output, para2, objIdx);
        return (obj2 - obj1) / (2* DERIVE_STEP);

}

MatrixXd jacobian(const VectorXd& input, const VectorXd& output, const VectorXd& params)  {
    int rowN = input.rows();
    int colN = params.rows();
    
    MatrixXd Jac(rowN, colN);

    for(int i=0;i< rowN; i++) {
        for(int j = 0;j< colN; j++) {
            Jac(i,j) = Deriv(input, output, i, params, j);
        }
    }
    cerr << "endof of jacobian. with size: "  << Jac.size() << std::endl;

    return Jac;
}

void gaussNewTon(const VectorXd& input, const VectorXd& output, VectorXd& params) {
    int errN = input.rows();
    int paraN = params.rows();
    VectorXd obj(errN);
    double last_sum = 0;
    
    int iterC = 0;
    while(iterC < MAX_ITER) {
        obj = objF(input, output, params);
        double sum  = 0;
        sum = sqerror(obj);
        cerr << "iterator index:" <<  iterC << std::endl;
        cerr << "param: " <<  params << std::endl;
        cerr << "sq error : " << sum << std::endl;
        if (fabs(sum - last_sum) <= 1e-12) {
            break;
        }
        last_sum = sum;
        MatrixXd  Jac = jacobian(input, output, params);
        VectorXd    delta(paraN);
        MatrixXd jtj = Jac.transpose() * Jac;
        delta = jtj.inverse() * Jac.transpose() * obj;
        params  -= delta;
        iterC++;
    }
}



double maxMatrixDiagonale(const MatrixXd& H) {
    int max = 0;
    for(int i =0;i<H.rows(); i++) {
        if(H(i,i) > max)
            max = H(i,i);
    }

    return max;
}



// L(h) = F(X) + h^t*J^t*f + h^t*j*h/2
// deltaL = h^t * (u*h-g)/2
//  -g = (H+u*I)*h
double linearDeltal(const VectorXd& step, const VectorXd& grad, const double u) {
    double L = step.transpose() * (u * step - grad);
    return L/2;
}


void LevelM(const VectorXd& input, const VectorXd& output, VectorXd& params) {
    int errN = input.rows();
    int paraN = params.rows();

    VectorXd obj = objF(input, output, params);
    MatrixXd Jac = jacobian(input, output, params);
    MatrixXd A = Jac.transpose() * Jac;
    VectorXd grad  = Jac.transpose() * obj;

    double tao  = 1e-3;
    long long v = 2;
    double eps1 = 1e-12, eps2 = 1e-12;
    double u = tao * maxMatrixDiagonale(A);
    bool found = grad.norm() <= eps1;

    cerr << "Entering lm algorithm loop" << std::endl;
    if(found)  {
        cerr << "Grad norm is too small : " << grad.norm() << std::endl;
        return;
    }
        
    double last_sum = 0;
    int iterC = 0;
    while(iterC < MAX_ITER) {
        VectorXd obj = objF(input, output, params);
        MatrixXd Jac = jacobian(input, output, params);
        MatrixXd A = Jac.transpose() * Jac;
        VectorXd grad = Jac.transpose() * obj;
        if (grad.norm() <= eps1)
        {
            cerr << "Stop g(x) = 0 for a local minimum " << endl;
            break;
        }

        VectorXd step = (A + MatrixXd::Identity(paraN, paraN)).inverse() * grad;
        if (step.norm() <= eps2*(params.norm() + eps2) )
        {
            cerr << "stop due to change in x is too small" << std::endl;
            break;
        }
        VectorXd paraNew(params.rows());
        paraNew = params - step;

        obj = objF(input, output, params);

        VectorXd obj_new = objF(input, output, paraNew);
        double deltaF = sqerror(obj) - sqerror(obj_new);
        double deltaL = linearDeltal(-1*step, grad, u);

        double roi = deltaF/ deltaL;
        if (roi > 0)
         {
             params = paraNew;
             u *= std::max(1/3.0, 1-pow(2*roi -1, 3));
         } else {
             u = u*v;
             v = v*2;
         }
         iterC ++;
         cerr << "iter no: " << iterC 
            << " roi " << roi 
            << "u : " << u
            << "v: " << v
            << "step " << step
            << std::endl;

    }

}



// reference: https://blog.csdn.net/stihy/article/details/52737723

void dogleg(const VectorXd& input, const VectorXd& output, VectorXd& params) {
    cerr << "dogleg begin. " << std::endl;
    int errN = input.rows();
    int paraN = params.rows();
    
    VectorXd obj = objF(input, output, params);
    
    auto x = jacobian(input, output, params);
    cerr << "jacobian no output assign" << std::endl;
    MatrixXd Jac = jacobian(input, output, params);
    cerr << "jacobian  ..." << std::endl;
    VectorXd grad = Jac.transpose() * obj;
    cerr << "debug ..." << std::endl;
    double eps1 = 1e-12,eps2 = 1e-12, eps3=1e-12;
    double radius = 1.0;
    bool found = obj.norm() <= eps3 || grad.norm() <= eps1;
    if(found)   
        return;
    double last_sum = 0;
    int iterC = 0;
    cerr << "begin to doglog iter." << std::endl;
    while(iterC < MAX_ITER) {
        VectorXd obj = objF(input, output, params);
        MatrixXd Jac = jacobian(input, output, params);
        VectorXd grad = Jac.transpose() * obj;
        if(grad.norm() < eps1) {
            cerr << "stop F'(x) = g(x) = 0  for a global minimizer optimizer." << std::endl;
            break;
        }

        if (obj.norm() < eps3)  {
            cerr << "stop for f(x) too small" << std::endl;
            break;
        }

        double alpha = grad.squaredNorm() / (Jac* grad).squaredNorm();
        VectorXd stepest_desc = -alpha * grad;
        VectorXd gauss_newton = (Jac.transpose()* Jac).inverse() * Jac.transpose() * obj * (-1);


        double beta = 0;

        VectorXd dog_leg(params.rows());

        if(gauss_newton.norm() <= radius) {
            dog_leg = gauss_newton;
        } else if(alpha * stepest_desc.norm()>= radius) {
            dog_leg = (radius/stepest_desc.norm()) * stepest_desc;
        } else {
            VectorXd a = alpha* stepest_desc;
            VectorXd b = gauss_newton;
            double c = a.transpose() * (b - a);
            beta = (sqrt(c*c + (b-a).squaredNorm()*(radius*radius - a.squaredNorm())) -c )/
                (b-a).squaredNorm();
            dog_leg = alpha * stepest_desc +  beta * (gauss_newton - alpha * stepest_desc);

        }

        cerr << "dog leg: " << dog_leg << std::endl;

        if(dog_leg.norm() <= eps2 * (params.norm() + eps2)) {
            cerr << "stop due to change of x is too small" << std::endl;
            break;
        }

        VectorXd new_params(params.rows());
        new_params = params + dog_leg;
        obj = objF(input, output, params);
        VectorXd obj_new = objF(input, output, new_params);
        double deltaF = sqerror(obj) - sqerror(obj_new);
        double deltaL = 0;

        if (gauss_newton.norm() <= radius) {
            deltaL  = sqerror(obj);
        } else if (alpha * stepest_desc.norm() >= radius) {
            deltaL = radius*(2* alpha * grad.norm() - radius) / (2.0* alpha);
            //deltaL = (radius / stepest_desc.norm())* stepest_desc;
        } else {
            VectorXd a = alpha * stepest_desc;
            VectorXd b = gauss_newton;
            double c = a.transpose() * (b - a);
            beta = (sqrt(c*c + (b-a).squaredNorm() * (radius*radius - a.squaredNorm())) -c ) 
                /  (b-a ).squaredNorm();
            deltaL = alpha * (1-beta)*(1-beta)*grad.squaredNorm()/2.0 + beta* (2.0-beta)*sqerror(obj);
        }

        double roi  = deltaF/ deltaL;
        if (roi > 0) {
            params = new_params;
        } 
        if (roi > 0.75) {
            radius = std::max(radius, 3.0*dog_leg.norm());

        } else if (roi < 0.25) {
            radius  = radius / 2.0;
            if (radius<= eps2 * (params.norm() + eps2)) {
                cerr << "trust regin too small" << std::endl;
                break;
            }
        }
        iterC++;
    }
}

int main() {
    int paraN = 4;
    int total_d = 100;
    VectorXd input(total_d);
    VectorXd output(total_d);

    Param para;
    cerr << "pa: " << para.A << std::endl;
    for(int i=0;i< total_d; i++) {
        double x = 20.0*((random() %1000)/1000) - 10.0;
        double deltaY = 2.0* (random() % 1000)/1000.0;
        double y = para.A*sin(para.B*x) + para.C * cos(para.D*x) + deltaY;
        input(i) = x;
        output(i) = y;
    }

    VectorXd para_gauss(paraN);
    para_gauss << 1.5 , 1.4 , 6.2 , 1.7 ;
    VectorXd para_lm = para_gauss;
    VectorXd para_dl = para_gauss;
    
    //
    //gaussNewTon(input, output, para_gauss);
     //LevelM(input, output, para_lm);
    dogleg(input, output, para_dl);

    cerr << "gauss newton para: " << endl << para_gauss << std::endl;
    cerr << "lm paras: " << endl  << para_lm << std::endl;
    cerr << "dog -leg paras: " << endl << para_dl << std::endl;
    return 1;
}