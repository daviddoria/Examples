#include <iostream>

#include <vnl/vnl_vector.h>
#include <vnl/vnl_matrix.h>
#include <vnl/algo/vnl_matrix_inverse.h>
#include <vnl/vnl_double_3x3.h>

vnl_double_3x3 rank2_approximate(vnl_double_3x3 const& F);

int main()
{
  /*
  octave:1> A=[1 2; 4 3; 6 3];
  octave:2> b=[4;5;6];
  octave:3> inv(A)*b
  error: inverse: argument must be a square matrix
  octave:3> pinv(A)*b
  ans =

    0.056338
    1.781690
  */

  vnl_matrix<double> A(3,2);
  std::cout << "Rows: " << A.rows() << std::endl;
  std::cout << "Cols: " << A.columns() << std::endl;
  A(0,0) = 1;
  A(0,1) = 2;
  A(1,0) = 4;
  A(1,1) = 3;
  A(2,0) = 6;
  A(2,1) = 3;
  std::cout << "A = " << A << std::endl;

  vnl_vector< double > b(3);
  b(0) = 4;
  b(1) = 5;
  b(2) = 6;

  std::cout << "b = " << b << std::endl;

  //Ax - b = 0, solution should be x = (.0563, 1.7817)
  vnl_vector< double > x(3);
  x = vnl_matrix_inverse<double>(A) * b;

  std::cout << "x = " << x << std::endl;

  return 0;
}


vnl_double_3x3 rank2_approximate(vnl_double_3x3 const& F)
{
  // Compute singular value decomposition of F
  vnl_svd<double> svd (F);

  // Set smallest singular value to 0
  svd.W(2,2) = 0;

  // Recompose vnl_svd<double> into UWV^T
  return vnl_double_3x3(svd.recompose());
}
