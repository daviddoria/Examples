#include "cv.h"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/core/core.hpp"

#include <iostream>

int main(int, char*[])
{
  // Create a matrix
  cv::Mat myMatrix3(3,3,CV_32FC1);
  std::cout << "Input:" << std::endl;
  for(unsigned int i = 0; i < 3; i++)
    {
    for(unsigned int j = 0; j < 3; j++)
      {
      myMatrix3.at<float>(i,j) = 2.1;
      }
    }

  std::cout << myMatrix3 << std::endl;

  cv::Mat myMatrix4(4,4, myMatrix3.type());
  std::cout << myMatrix4 << std::endl;

  for(unsigned int i = 0; i < 3; i++)
    {
    for(unsigned int j = 0; j < 3; j++)
      {
      myMatrix4.at<float>(i,j) = myMatrix3.at<float>(i,j);
      }
    }

  std::cout << myMatrix4 << std::endl;

  return 0;
}

