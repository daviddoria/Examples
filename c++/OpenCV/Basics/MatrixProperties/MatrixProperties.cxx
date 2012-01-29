#include "cv.h"

#include <iostream>

int main()
{
  cv::Mat A(3,3,CV_32FC1);

  std::cout << A.depth() << std::endl; // float seems to be depth 5

  cv::Mat B(3,3,CV_64FC1);

  std::cout << B.depth() << std::endl; // float seems to be depth 6

  return 0;
}

