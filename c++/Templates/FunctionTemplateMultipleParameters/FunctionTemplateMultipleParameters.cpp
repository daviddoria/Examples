#include <iostream>

template<typename T, typename U>
void PrintSomething(T something, U somethingElse)
{
  std::cout << something << std::endl;
}


int main(int, char*[])
{
  PrintSomething(1.0, 1);

  return 0;
}
