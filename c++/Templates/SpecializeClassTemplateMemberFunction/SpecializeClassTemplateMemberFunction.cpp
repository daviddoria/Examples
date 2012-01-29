#include <iostream>

template <typename T>
class MyClass
{
public:

  void Test(T)
  {
    std::cout << "Normal" << std::endl;
  }
};

template<> void MyClass<float>::Test(float arg)
{
  std::cout << "Specialization!" << std::endl;
}

int main(int argc, char* argv[])
{
  MyClass<int> p;
  int a;
  p.Test(a);

  MyClass<float> p2;
  float b;
  p2.Test(b);

  return 0;
}
