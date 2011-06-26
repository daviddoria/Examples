#include <iostream>

#include <vnl/vnl_vector.h>
#include <vnl/vnl_sparse_matrix.h>
#include <vnl/algo/vnl_sparse_lu.h>

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

  vnl_sparse_matrix<double> A(3,3);
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

  vnl_vector< double > b(3);
  b(0) = 1;
  b(1) = -2;
  b(2) = 0;

  std::cout << "b = " << b << std::endl;

  //Ax - b = 0, solution should be x = (1 -2 -2)
  vnl_vector< double > x(3);

  vnl_sparse_lu linear_solver(A, vnl_sparse_lu::estimate_condition);
  linear_solver.solve(b,&x);
  double det = linear_solver.determinant();
  double rcond = linear_solver.rcond();
  double upbnd = linear_solver.max_error_bound();

  std::cout << "x = " << x << std::endl;

  return 0;
}

