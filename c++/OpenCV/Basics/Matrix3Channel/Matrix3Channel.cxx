#include "cv.h"

#include <iostream>

int main()
{

  cv::Mat myMatrix(3,3,CV_32FC3);
  std::cout << "Input:" << std::endl;
  for(unsigned int i = 0; i < 3; i++)
    {
    for(unsigned int j = 0; j < 3; j++)
      {
      cv::Point3f myPoint;
      myPoint.x = 1;
      myPoint.y = 2;
      myPoint.z = 3;
      myMatrix.at<cv::Point3f>(i,j) = myPoint;
      }
    }

  std::cout << "Output:" << std::endl;

  for(unsigned int i = 0; i < 3; i++)
    {
    for(unsigned int j = 0; j < 3; j++)
      {
      std::cout << myMatrix.at<cv::Point3f>(i,j) << std::endl;
      }
    }

  return 0;
}

