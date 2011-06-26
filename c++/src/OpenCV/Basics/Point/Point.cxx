#include "cv.h"

#include <iostream>

int main()
{
  cv::Point3f myPoint;
    
  myPoint.x = 1;
  myPoint.y = 2;
  myPoint.z = 3;
  std::cout << myPoint << std::endl;
  
  return 0;
}