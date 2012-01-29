#include <iostream>

template<typename T>
class Point
{
public:
  template<typename S>
  void GetX(const S& test);

  template<typename S>
  void NoParameter(){};
};

template<typename T>
template<typename S>
void Point<T>::GetX(const S& test)
{
  std::cout << test << std::endl;
}


int main(int argc, char* argv[])
{
  Point<int> a;
  a.GetX(2);

  a.NoParameter<int>();
  return 0;
}
