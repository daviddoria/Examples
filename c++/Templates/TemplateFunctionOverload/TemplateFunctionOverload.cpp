#include <iostream>

template<typename T>
void PrintSomething(T something);

template<typename T>
void PrintSomething(T something, int a);

int main(int, char*[])
{
  double a = 1.2;
  PrintSomething(a);

  // Sometimes have to do this:
  PrintSomething<double>(a);

  PrintSomething<double>(a, 2);

  return 0;
}

template<typename T>
void PrintSomething(T something)
{
  std::cout << something << std::endl;
}

template<typename T>
void PrintSomething(T something, int a)
{
  std::cout << something << " " << a << std::endl;
}

template void PrintSomething<double>(double);
