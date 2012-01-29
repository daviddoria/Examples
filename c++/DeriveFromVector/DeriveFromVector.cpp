#include <iostream>
#include <vector>

class MyClass : public std::vector<double>
{

};

int main(int argc, char *argv[])
{
  MyClass A;

  A.push_back(2.0);
  A.push_back(3.0);

  std::cout << A[0] << std::endl;

  return 0;
}
