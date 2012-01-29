#include <iostream>

class TestClass
{
public:

  void SetA(const int a_in) {a = a_in;}
  int& GetARef() {return a;}
  int GetA() {return a;}

  void OutputA() { std::cout << "A: " << a << std::endl;}

private:
  int a;
};

int main(int argc, char *argv[])
{
  TestClass test;
  test.SetA(3);

  int b = test.GetARef();
  std::cout << "b: " << b << std::endl;
  b = 5;
  test.OutputA();

  int& c = test.GetARef();
  std::cout << "c: " << c << std::endl;
  c = 5;
  test.OutputA();

//   int& d = test.GetA(); // Can't do this.
//   std::cout << "d: " << d << std::endl;
//   d = 5;
//   test.OutputA();
  return 0;
}
