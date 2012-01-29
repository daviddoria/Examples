#include <iostream>

#include <Eigen/Dense>

#include <unsupported/Eigen/NonLinearOptimization>

// LM minimize for sum_i (f_i(x))^2

struct MyFunctor
{
  int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
  {
    // Implement y = (x-5)^2 (remember, operator() should return the value BEFORE it is squared.
    fvec(0) = x(0) - 5.0;
    return 0;
  }

  int df(const Eigen::VectorXf &x, Eigen::MatrixXf &fjac) const
  {
    Eigen::VectorXf epsilon(1);
    epsilon(0) = 1e-5;

    Eigen::VectorXf fvec1(1);
    operator()(x + epsilon, fvec1);
    Eigen::VectorXf fvec2(1);
    operator()(x - epsilon, fvec2);
    fjac = (fvec1 - fvec2)/2.0f;
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

  MyFunctor functor;
  Eigen::LevenbergMarquardt<MyFunctor, float> lm(functor);

  lm.parameters.ftol = 1e-6;
  lm.parameters.xtol = 1e-6;
  lm.parameters.maxfev = 10; // Max iterations
  
  lm.minimize(x);

  std::cout << "x that minimizes the function: " << x << std::endl;

  return 0;
}
