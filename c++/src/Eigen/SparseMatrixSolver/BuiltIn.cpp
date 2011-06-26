#include <iostream>

#include <Eigen/Sparse>

int main(int argc, char *argv[])
{
  Eigen::SparseMatrix<float> A(3,3);
  A.insert(0,0) = 3;
  A.insert(0,1) = 2;
  A.insert(0,2) = -1;
  A.insert(1,0) = 2;
  A.insert(1,1) = -2;
  A.insert(1,2) = 4;
  A.insert(2,0) = -1;
  A.insert(2,1) = .5;
  A.insert(2,2) = -1;

  std::cout << A << std::endl;

  // Eigen::VectorXd b(3); //double
  Eigen::VectorXf b(3); //float
  b(0) = 1;
  b(1) = -2;
  b(2) = 0;

  Eigen::VectorXf x(3);
  // other choice is SimplicialCholeskyLDLt
  //x = SimplicialCholesky<SparseMatrix<Scalar>, Lower>().setMode(SimplicialCholeskyLLt).compute(A).solve(b);

  // Can only use this for a symmetric matrix!
  x = Eigen::SimplicialCholesky<Eigen::SparseMatrix<float> >(A).solve(b);

  std::cout << "Solution: " << x << std::endl;
  // Solution should be (1, -2, -2)
  // The solution produced is NOT correct because the matrix is not symmetric

  return 0;
}

