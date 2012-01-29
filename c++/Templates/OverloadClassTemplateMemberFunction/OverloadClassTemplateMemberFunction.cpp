#include <iostream>

template <typename T>
class Point
{
public:

  //double Test(typename std::enable_if<!std::is_same<T, double>::value, T>::type) {return 1.0;} // error: no type named 'type'

//   template<typename U>
//   double Test(typename std::enable_if<!std::is_same<T, double>::value, T>::type) {return 1.0;} // error: no type named 'type'

  // If you use T for the enable_if (above), it's not dependent on U, and gets evaluated as soon as the class is instantiated.
  // Using U makes it dependent on U and the enable_if gets evaluated when U is provided.
  template<typename U=T>
  double Test(typename std::enable_if<!std::is_same<U, double>::value, U>::type) {return 1.0;} // error: no type named 'type'

  double Test(double) {return 1.0;}
};

int main(int argc, char* argv[])
{
  Point<int> p;
  int a = 0;
  p.Test(a);

  Point<double> p2;
  double b = 0.0d;
  p2.Test(b);
  return 0;
}
