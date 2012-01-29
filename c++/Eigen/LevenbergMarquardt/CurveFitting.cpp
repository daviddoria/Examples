#include <iostream>

#include <Eigen/Dense>

#include <unsupported/Eigen/NonLinearOptimization>

// LM minimize for the model y = a x + b
typedef std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > Point2DVector;

Point2DVector GeneratePoints();

struct MyFunctor
{
  int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
  {
    // "a" in the model is x(0), and "b" is x(1)
    for(unsigned int i = 0; i < this->Points.size(); ++i)
      {
      fvec(i) = x(0) * this->Points[i](0) + x(1);
      }
    return 0;
  }

  int df(const Eigen::VectorXf &x, Eigen::MatrixXf &fjac) const
  {
    // This problem is R^2 -> R^(fvec.rows()), so the dimension of the Jacobian is fvec.rows() x 2
    
    float epsilon = 1e-5;
    Eigen::VectorXf epsilonVector(2);

    Eigen::VectorXf f(this->Points.size());
    operator()(epsilonVector, f);
    
    for(unsigned int parameter = 0; parameter < 2; parameter++)
      {
      epsilonVector = x;
      epsilonVector(parameter) += epsilon;

      Eigen::VectorXf fForward(2);
      operator()(epsilonVector, fForward);

      fjac.col(parameter) = (fForward - f)/epsilon;
      }

    return 0;
  }

  Point2DVector Points;
  
  int inputs() const { return 2; } // There are two parameters of the model
  int values() const { return this->Points.size(); } // The number of observations
};

Point2DVector GeneratePoints()
{
  Point2DVector points;
  // Model y = 2*x + 5 with some noise
  for(unsigned int i = 0; i < 50; ++i)
    {
    float x = static_cast<float>(i);
    Eigen::Vector2d point;
    point(0) = x;
    point(1) = 2.0f * x + 5.0 + drand48()/10.0f;
    points.push_back(point);
    }

  return points;
}

int main(int argc, char *argv[])
{
  Point2DVector points = GeneratePoints();
  
  Eigen::VectorXf x(2);
  x.fill(1.0f);

  MyFunctor functor;
  functor.Points = points;
  Eigen::LevenbergMarquardt<MyFunctor, float> lm(functor);

  lm.parameters.ftol = 1e-6;
  lm.parameters.xtol = 1e-6;
  lm.parameters.maxfev = 1000; // Max iterations

  lm.minimize(x);

  std::cout << "x that minimizes the function: " << x << std::endl;

  return 0;
}
