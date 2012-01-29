#include "cv.h"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>

int main(int argc, char*argv[])
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

  std::cout << myMatrix << std::endl;

  cv::imwrite("matrix.jpg", myMatrix);

  cv::Mat inputMatrix = cv::imread("matrix.jpg", 1);

  std::cout << inputMatrix << std::endl;

  return 0;
}

