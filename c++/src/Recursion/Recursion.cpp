#include <iostream>

void OutputNumbers(unsigned int number);

int main(int argc, char *argv[])
{
  std::cout << "OutputNumbers(1)" << std::endl;
  OutputNumbers(1);

  std::cout << "OutputNumbers(6)" << std::endl;
  OutputNumbers(6);

  return 0;
}

void OutputNumbers(unsigned int number)
{
  std::cout << number << std::endl;

  // base case
  if(number == 0)
  {
    return;
  }
  
  OutputNumbers(number-1);
}