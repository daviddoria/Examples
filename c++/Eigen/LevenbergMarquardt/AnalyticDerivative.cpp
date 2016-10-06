#include <cmath>
#include <iostream>

#include <Eigen/Dense>

#include <unsupported/Eigen/NonLinearOptimization>

// Implement y = (x-5)^2
struct MyFunctor
{
    int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
    {
    // The squaring of each term is automatically done by the algorithm, so we just need to provide f(x) = x - 5

    fvec(0) = x(0) - 5.0;
    return 0;
    }

    int df(const Eigen::VectorXf &x, Eigen::MatrixXf &fjac) const
    {
    // Implement dy/dx = d/dx (x-5) = 1
    fjac(0) = 1.0f;
    return 0;
    }

    int inputs() const { return 1; }// 'inputs' is the dimension of x (the number of unknowns)
    int values() const { return 1; } // 'values' is the number of "constraints" (not sure what this means?)
};


int main(int argc, char *argv[])
{
    MyFunctor functor;

    Eigen::VectorXf x(1); // length 1 (a scalar)
    x(0) = 2;

    // For demo explanation only, evaluate f(2)
    {
    std::cout << "x = " << x << std::endl;

    Eigen::VectorXf fvalue(1); // length 1 (a scalar)
    functor(x, fvalue);
    std::cout << "f(" << x(0) << ") = " << std::pow(fvalue(0),2) << std::endl;
    }

    Eigen::LevenbergMarquardt<MyFunctor, float> levenbergMarquardt(functor);
    Eigen::VectorXf xmin = x; // initial value
    levenbergMarquardt.minimize(xmin);

    std::cout << "x that minimizes the function: " << xmin << std::endl;

    return 0;
}
