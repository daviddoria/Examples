#include "itkMemoryProbe.h"

#include <iostream>
#include <string>

void LongFunction();

int main(int, char *[])
{
  itk::MemoryProbe memoryProbe;

  memoryProbe.Start();
  LongFunction();

  memoryProbe.Stop();
  std::cout << "Mean: " << memoryProbe.GetMean() << std::endl;
  std::cout << "Total: " << memoryProbe.GetTotal() << std::endl;

  memoryProbe.Start();
  LongFunction();

  memoryProbe.Stop();
  std::cout << "Mean: " << memoryProbe.GetMean() << std::endl;
  std::cout << "Total: " << memoryProbe.GetTotal() << std::endl;

  return EXIT_SUCCESS;
}

void LongFunction()
{
  for(int i = 0; i < 1e8; i++)
  {
    double a = 0;
  }
}