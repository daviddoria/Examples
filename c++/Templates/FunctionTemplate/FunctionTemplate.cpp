#include <iostream>

template<typename T>
void PrintSomething(T something)
{
  std::cout << something << std::endl;
}

template<typename T>
void NoParameters()
{
  std::cout << "NoParameters" << std::endl;
}

int main(int, char*[])
{
  PrintSomething(1.0);

  // Sometimes have to do this:
  PrintSomething<double>(1.0);

  NoParameters<double>();
  
  return 0;
}
