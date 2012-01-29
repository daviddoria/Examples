#include <iostream>

// template <typename T>
// class Point
// {
// public:
//   double Test(T) {}
//   double Test(double) {}
// };

template <typename T>
class Point
{
public:

  double Test(typename std::enable_if<!std::is_same<T, double>::value, T>::type) {return 1.0;}
  double Test(double) {return 1.0;}
};

int main(int argc, char* argv[])
{
  Point<int> p;
  int a;
  p.Test(a);

  Point<double> p2;
  double b;

  return 0;
}
