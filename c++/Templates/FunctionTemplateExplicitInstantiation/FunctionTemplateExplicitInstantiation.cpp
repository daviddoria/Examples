#include <iostream>

template<typename T>
void PrintSomething(T something)
{
  std::cout << something << std::endl;
}

template void PrintSomething<double>(double);

int main(int, char*[])
{
  PrintSomething(1.2);

  return 0;
}
