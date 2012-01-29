#include <iostream>

template <int N>
void TestOutput()
{
  std::cout << N << std::endl;
}

int main(int argc, char* argv[])
{
  TestOutput<2>();
  
  // Can't do this:
//   int a = 3;
//   TestOutput<a>();
    
  return 0;
}
