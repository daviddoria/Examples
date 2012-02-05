#include <iostream>
#include <vector>

class Person
{
public:
  const int* const GetConstPointerToConstId(){};
  const int* GetPointerToConstId(){};
  int* const GetConstPointerToId(){};
};

int main()
{

  return 0;
}