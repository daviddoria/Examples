#include <iostream>

#include <vnl/vnl_matrix.h>
#include <vnl/algo/vnl_adjugate.h>

int main()
{
  vnl_matrix<double> A(3,3);
  A.set_identity();

  vnl_matrix<double> B = vnl_adjugate(A);

  return 0;
}
