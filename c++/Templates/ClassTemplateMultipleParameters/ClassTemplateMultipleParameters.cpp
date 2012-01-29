#include <iostream>

template<typename T, typename U>
class Point
{
public:
  void MyFunction();
};

template< typename T, typename U >
void Point<T, U>::MyFunction()
{

}

int main(int argc, char* argv[])
{
  Point<int, double> point;
  point.MyFunction();
  return 0;
}