#ifndef POINT_H
#define POINT_H

class Point
{
  double x,y,z;

public:
  template <typename T>
  double Add(const T a, const T b);

  // Apparently you can't specialize a member function template without the class being a class template.
  // error: explicit specialization in non-namespace scope ‘class Point’
//   template <>
//   double Add<int>(const int a, const int b);

  // Prefer to simply overload:
  double Add(const int a, const int b);
};

template <typename T>
double Point::Add(const T a, const T b)
{
  std::cout << "generic" << std::endl;
  return a + b;
}

double Point::Add(const int a, const int b)
{
  std::cout << "specialization" << std::endl;
  return a + b;
}

#endif