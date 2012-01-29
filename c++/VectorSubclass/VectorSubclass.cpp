#include <iostream>

#include <vector>

class MyClass : public std::vector<int>
{
public:
  void Output() {std::cout << "Test" << std::endl;}
};

int main(int argc, char* argv[])
{
  
  MyClass test;
  test.Output();
  
  test.push_back(2);
  return 0;
}
