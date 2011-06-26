#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Sparse>

int main(int argc, char *argv[])
{

  Eigen::MatrixXd D(2,2);
  D(0,0) = 3;
  D(1,0) = 2.5;
  D(0,1) = 1;
  D(1,1) = 5;

  std::cout << D << std::endl;

  std::cout << D.adjoint() << std::endl;

  Eigen::SparseMatrix<double> S(4,2);
  S.insert(0,0) = 3;
  S.insert(1,0) = 2.5;
  S.insert(0,1) = 1;
  S.insert(1,1) = 5;

  std::cout << S << std::endl;

  std::cout << S.adjoint() << std::endl;

  return 0;
}
