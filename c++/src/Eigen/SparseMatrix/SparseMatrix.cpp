#include <iostream>

#include <Eigen/Sparse>

int main(int argc, char *argv[])
{
  Eigen::SparseMatrix<float> mat(3,3);
  mat.insert(0,0) = 5;

  std::cout << mat << std::endl;

  return 0;
}
