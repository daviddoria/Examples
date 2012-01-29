#include <iostream>

#include <Eigen/Dense>

#include <unsupported/Eigen/NonLinearOptimization>

// LM minimize for sum_i (f_i(x))^2

struct MyFunctor
{
  int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
  {
    // Implement y = x^2 (remember, operator() should return the value BEFORE it is squared.
    fvec(0) = x(0) - 5.0;
    return 0;
  }

  int df(const Eigen::VectorXf &x, Eigen::MatrixXf &fjac) const
  {
    // Implement dy/dx = 2*x
    fjac(0) = 2.0f * x(0);
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
  lm.minimize(x);

  std::cout << "x that minimizes the function: " << x << std::endl;

  return 0;
}
