#include <iostream>

template<typename T>
void PrintSomething(T something)
{
  std::cout << something << std::endl;
}

int main(int, char*[])
{
  PrintSomething(1.2);
  
  PrintSomething(2);

  return 0;
}
