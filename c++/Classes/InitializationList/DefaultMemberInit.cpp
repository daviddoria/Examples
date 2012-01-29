#include <iostream>

class Point
{
  public:
    Point(int NewValue) : a(NewValue) {}
    int a;
};

int main(int argc, char* argv[])
{
  Point P(2);
  std::cout << P.a << std::endl;
  return 0;
}
