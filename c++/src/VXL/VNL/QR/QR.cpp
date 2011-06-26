#include <iostream>
#include <fstream>

#include "vnl/algo/vnl_qr.h"

int main()
{

  vnl_qr<TScalarType> qr( vnl_matrix_1 );

  vnl_matrix< TScalarType > vnl_matrix_2 = qr.solve( vnl_matrix_3 );
  // or
  vnl_matrix< TScalarType > vnl_matrix_Q = qr.Q();
  vnl_matrix< TScalarType > vnl_matrix_R = qr.R();

  return 0;
}
