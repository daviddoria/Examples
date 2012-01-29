#include <iostream>

template<typename T>
class Point
{
public:
  static int a;
};

template<typename T>
int Point<T>::a = 5;

int main(int argc, char* argv[])
{

  return 0;
}
