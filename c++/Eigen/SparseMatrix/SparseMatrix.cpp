#include <iostream>

#include <Eigen/Sparse>

int main(int argc, char *argv[])
{
  //Eigen::SparseMatrix<float> mat(3,3);
  //mat.insert(0,0) = 5;
  
  Eigen::SparseMatrix<float> mat(5,5);
  mat.insert(1,0) = 22;
  mat.insert(2,0) = 7;
  mat.insert(0,1) = 3;
  mat.insert(2,1) = 5;
  mat.insert(4,2) = 14;
  mat.insert(2,3) = 1;
  mat.insert(1,4) = 17;
  mat.insert(4,4)= 8;
  std::cout << mat << std::endl;
  
  // column pointers: 0 2 4 5 6 means that nonzero entries [0,2) is in the first col,   [2,4] is in the second col,
  
  return 0;
}
