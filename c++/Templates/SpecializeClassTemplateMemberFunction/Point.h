#ifndef POINT_H
#define POINT_H

template <typename T>
class Point
{
  T x,y,z;

public:
  double Add();
};

template <>
class Point <float>
{
  float x,y,z;

public:
  double Add();
};

template <typename T>
double Point<T>::Add()
{
  std::cout << "General Add()" << std::endl;
  return 2.0 + 4.3;
}

double Point<float>::Add()
{
  std::cout << "Float Add()" << std::endl;
  return 2.0 + 4.3;
}

#endif