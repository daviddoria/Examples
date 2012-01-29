#include <iostream>

#include <Eigen/Sparse>
#include <Eigen/UmfPackSupport>
#include <Eigen/SparseExtra>

// On Fedora make sure to install the packages suitesparse*

// Everything MUST be double (or you will get "no matching function call" error)

int main(int argc, char *argv[])
{
  Eigen::SparseMatrix<double> A(3,3);
  A.insert(0,0) = 3;
  A.insert(0,1) = 2;
  A.insert(0,2) = -1;
  A.insert(1,0) = 2;
  A.insert(1,1) = -2;
  A.insert(1,2) = 4;
  A.insert(2,0) = -1;
  A.insert(2,1) = .5;
  A.insert(2,2) = -1;

  //std::cout << A(2,2) << std::endl;
  //std::cout << A[2,2] << std::endl;

  std::cout << "A: " << A << std::endl;

  Eigen::VectorXd b(3);
  b(0) = 1;
  b(1) = -2;
  b(2) = 0;

  std::cout << "b: " << b << std::endl;

  Eigen::VectorXd x(3);
  // Solution should be (1 -2 -2)

  // solve Ax = b using UmfPack:
  Eigen::SparseLU<Eigen::SparseMatrix<double>,Eigen::UmfPack> lu_of_A(A);
  if(!lu_of_A.succeeded())
  {
    // decomposiiton failed
    return -1;
  }
  if(!lu_of_A.solve(b,&x))
  {
    // solving failed
    return -1;
  }

  std::cout << "x: " << x << std::endl;

  return 0;
}

