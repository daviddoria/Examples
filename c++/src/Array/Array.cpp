#include <iostream>

double* ReturnArray();

int main()
{
  double a[3];
  a[0] = 0.5;
  a[1] = 1.5;
  a[2] = 2.5;

  std::cout << a[0] << " " << a[1] << " " << a[2] << std::endl;

  double* A;
  A = ReturnArray();
  std::cout << A[0] << " " << A[1] << " " << A[2] << std::endl;
  delete A;

  return 0;
}


double* ReturnArray()
{
  double* A = new double[3];
  A[0] = 1.0;
  A[1] = 2.0;
  A[2] = 3.0;

  return A;
}