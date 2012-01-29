#include <iostream>
#include <vector>
#include <cstdlib>

#include <cmath>
#include <boost/thread.hpp>

void LongFunction();
		
int main(int argc, char* argv[])
{
  std::cout << "Main: Before thread call" << std::endl;
  
  boost::thread MyThread(&LongFunction);
  
  std::cout << "Main: After thread call" << std::endl;
  
  MyThread.join();
  
  return 0;
}

void LongFunction()
{
  std::cout << "Start LongFunction" << std::endl;
  unsigned int BigNum = 1e7;
  
  double temp;
  for(unsigned int i = 0; i < BigNum; i++)
  {
    temp = sin(i) / i;
  }
  std::cout << "End LongFunction" << std::endl;
}

