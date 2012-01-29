#include <iostream>

template<class T>
struct Point{
  typedef T type;
  T x;
  T GetX(){return x;}
};

template< class T >
typename Point<T>::type MyFunction( Point<T>&p ){
  typename Point<T>::type value = p.GetX();
  return value;
}

Point<double> A;

void MyFunction();

int main(int argc, char* argv[])
{
  //A::type test = MyFunction( A );
  
  return 0;
}