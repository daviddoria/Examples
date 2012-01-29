#include <iostream>

template <typename T>
class Point
{
public:
  typedef T PointType;
  T x;
  T GetX(){return x;}
};

Point<double> A;

void MyFunction();

int main(int argc, char* argv[])
{
  MyFunction();

  return 0;
}

void MyFunction()
{
  A::PointType test = A.GetX();
  return 0;
}