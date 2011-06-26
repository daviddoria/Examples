#include "itkRealTimeClock.h"

#include <iostream>
#include <string>

int main(int argc, char *argv[])
{
  itk::RealTimeClock::Pointer clock = itk::RealTimeClock::New();
  std::cout << clock->GetTimeStamp() << std::endl;
  return EXIT_SUCCESS;
}