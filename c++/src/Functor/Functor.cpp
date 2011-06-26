#include <iostream>

struct Function
{
  virtual double operator() (const double x, const double& y) const = 0;
};

struct Add : public Function
{
  double operator() (const double x, const double& y) const
  {
    return x+y;
  }
};

struct Multiply : public Function
{
  double operator() (const double x, const double& y) const
  {
    return x*y;
  }
};

int main(int argc, char *argv[])
{
  Function* a;
  a = new Add;
  std::cout << (*a)(2.,3.) << std::endl;
  delete a;
  
  a = new Multiply;
  std::cout << (*a)(2.,3.) << std::endl;
  
  return 0;
}
