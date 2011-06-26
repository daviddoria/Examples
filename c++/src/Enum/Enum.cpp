#include <iostream>

enum PointType { GOOD, BAD, UNINFORMATIVE };
	
int main(int argc, char *argv[])
{
  PointType A = GOOD;
  
  if(A == GOOD)
  {
    std::cout << "good" << std::endl;
  }
  else
  {
    std::cout << "not good" << std::endl;
  }

  if(A == BAD)
  {
    std::cout << "not working" << std::endl;
  }
  else
  {
    std::cout << "working" << std::endl;
  }
	
  return 0;
}