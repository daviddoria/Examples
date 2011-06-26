#include "itkTimeProbe.h"

#include <iostream>
#include <string>

void LongFunction();

int main(int, char *[])
{
  itk::TimeProbe clock;

  clock.Start();
  LongFunction();

  clock.Stop();
  std::cout << "Mean: " << clock.GetMean() << std::endl;
  std::cout << "Total: " << clock.GetTotal() << std::endl;

  clock.Start();
  LongFunction();

  clock.Stop();
  std::cout << "Mean: " << clock.GetMean() << std::endl;
  std::cout << "Total: " << clock.GetTotal() << std::endl;

  return EXIT_SUCCESS;
}

void LongFunction()
{
  for(int i = 0; i < 1e8; i++)
  {
    double a = 0;
  }
}