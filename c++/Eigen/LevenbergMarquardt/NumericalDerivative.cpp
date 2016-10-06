#include <iostream>

#include <Eigen/Dense>

#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

// Implement y = (x-5)^2
struct MyFunctor
{
    typedef float Scalar;

    typedef Eigen::VectorXf InputType;
    typedef Eigen::VectorXf ValueType;
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic> JacobianType;

    enum {
        InputsAtCompileTime = Eigen::Dynamic,
        ValuesAtCompileTime = Eigen::Dynamic
    };

    int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
    {
        // We provide f(x) = x-5 because the algorithm will square this value internally
        fvec(0) = x(0) - 5.0;
        return 0;
    }

    int inputs() const { return 1; }// inputs is the dimension of x.
    int values() const { return 1; } // "values" is the number of f_i and
};

int main(int argc, char *argv[])
{
    Eigen::VectorXf x(1);
    x(0) = 2;
    std::cout << "x: " << x << std::endl;

    MyFunctor myFunctor;
    Eigen::NumericalDiff<MyFunctor> numericalDiffMyFunctor(myFunctor);
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<MyFunctor>, float> levenbergMarquardt(numericalDiffMyFunctor);

    levenbergMarquardt.parameters.ftol = 1e-6;
    levenbergMarquardt.parameters.xtol = 1e-6;
    levenbergMarquardt.parameters.maxfev = 10; // Max iterations

    Eigen::VectorXf xmin = x; // initialize
    levenbergMarquardt.minimize(xmin);

    std::cout << "x that minimizes the function: " << xmin << std::endl;

    return 0;
}
