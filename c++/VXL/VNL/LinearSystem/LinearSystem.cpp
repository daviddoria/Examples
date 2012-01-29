#include <iostream>

#include <vnl/vnl_vector.h>
#include <vnl/vnl_matrix.h>
#include <vnl/algo/vnl_matrix_inverse.h>
#include <vnl/vnl_double_3x3.h>

int main()
{
  /*
  octave:1> A=[3 2 -1; 2 -2 4; -1 .5 -1];
  octave:2> b=[ 1; -2; 0];
  octave:3> inv(A)*b
  ans =

    1.00000
    -2.00000
    -2.00000
  */

  vnl_matrix<double> A(3,3);
  std::cout << "Rows: " << A.rows() << std::endl;
  std::cout << "Cols: " << A.columns() << std::endl;
  A(0,0) = 3;
  A(0,1) = 2;
  A(0,2) = -1;
  A(1,0) = 2;
  A(1,1) = -2;
  A(1,2) = 4;
  A(2,0) = -1;
  A(2,1) = .5;
  A(2,2) = -1;
  std::cout << "A = " << A << std::endl;

  vnl_vector< double > b(3);
  b(0) = 1;
  b(1) = -2;
  b(2) = 0;

  std::cout << "b = " << b << std::endl;

  //Ax - b = 0, solution should be x = (1 -2 -2)
  vnl_vector< double > x(3);
  x = vnl_matrix_inverse<double>(A) * b;

  std::cout << "x = " << x << std::endl;

  return 0;
}

