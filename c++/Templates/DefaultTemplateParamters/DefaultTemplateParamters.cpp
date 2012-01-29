#include <iostream>

template <typename T = int>
class Test
{
  
};

int main(int argc, char* argv[])
{
  //Test a; // Can't do this
  Test<> a;

  return 0;
}
