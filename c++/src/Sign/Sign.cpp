#include <iostream>

int sign(int v)
{
return v > 0 ? 1 : (v < 0 ? -1 : 0);
}

int sign(double v)
{
return v > 0 ? 1 : (v < 0 ? -1 : 0);
}

int main (int, char *[])
{
  int a = 2;
  int b = -2;

  std::cout << a << " : " << sign(a) << std::endl;
  std::cout << b << " : " << sign(b) << std::endl;

  int c = 2.0;
  int d = -2.0;

  std::cout << c << " : " << sign(c) << std::endl;
  std::cout << d << " : " << sign(d) << std::endl;

  return 0;
}

