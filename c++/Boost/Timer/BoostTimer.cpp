#include <iostream>
#include <vector>
#include <cstdlib>
#include <cmath>

#include <boost/timer.hpp>

void LongFunction(const unsigned int BigNum);

int main(int, char* [])
{
  boost::timer timer;
  LongFunction(1e8);
  std::cout << timer.elapsed() << " seconds." << std::endl;
  timer.restart();
  LongFunction(1e8);
  std::cout << timer.elapsed() << " seconds." << std::endl;
  
  return 0;
}

void LongFunction(const unsigned int BigNum)
{
  double temp;
  for(unsigned int i = 0; i < BigNum; i++)
  {
    temp = sin(i) / i;
  }
}

