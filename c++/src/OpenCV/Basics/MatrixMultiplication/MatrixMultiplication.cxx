#include "cv.h"

#include <iostream>

int main()
{
  cv::Mat A(3,3,CV_32FC1);
  for(unsigned int i = 0; i < 3; i++)
    {
    for(unsigned int j = 0; j < 3; j++)
      {
      A.at<float>(i,j) = 1;
      }
    }

  std::cout << "A: " << A << std::endl;

  cv::Mat B(3,3,CV_32FC1);

  for(unsigned int i = 0; i < 3; i++)
    {
    for(unsigned int j = 0; j < 3; j++)
      {
      B.at<float>(i,j) = 2;
      }
    }

  std::cout << "B: " << B << std::endl;

  cv::Mat result = A * B;

  std::cout << "result: " << result << std::endl;

  return 0;
}

