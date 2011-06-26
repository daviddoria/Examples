#include "distributions.h"
#include <iostream>

int main()
{
  double k=scythe::gammafn(5.0);
  std::cout << "gamma(5.0) = " << k << std::endl;

  return 0;
}
