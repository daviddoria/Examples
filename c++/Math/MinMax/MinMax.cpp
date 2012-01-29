#include <iostream>
#include <cmath>

int main(int argc, char *argv[])
{

  {
  double a = 10.3;
  double b = 5.8;

  std::cout << "Min: " << std::min(a,b) << std::endl;
  std::cout << "Max: " << std::max(a,b) << std::endl;
  }
  
  //doesn't work
  /*
  {
  double a = 10.3;
  int b = 5;

  std::cout << "Min: " << std::min(a,b) << std::endl;
  std::cout << "Max: " << std::max(a,b) << std::endl;
  }
  */
  
  {
  double a = 10.3;
  int b = 5;

  std::cout << "Min: " << std::min(a,static_cast<double>(b)) << std::endl;
  std::cout << "Max: " << std::max(a,static_cast<double>(b)) << std::endl;
  }
  
  return 0;
  
}
