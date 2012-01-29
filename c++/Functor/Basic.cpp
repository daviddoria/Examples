#include <iostream>

struct MyMultiplies
{
  double operator() (const double x, const double& y) const
  {
    return x*y;
  }
};

int main(int argc, char *argv[])
{
  MyMultiplies a;
  std::cout << a(2.,3.) << std::endl;
  return 0;
}
