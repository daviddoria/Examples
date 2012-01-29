#include <iostream>

#include <vnl/vnl_sparse_matrix.h>

int main()
{
  vnl_sparse_matrix<double> A(3,3);
  A(0,0) = 1.0;

  // operator<< is not defined
  // identity is not defined
  return 0;
}
