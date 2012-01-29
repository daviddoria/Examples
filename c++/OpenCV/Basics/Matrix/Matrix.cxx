#include "cv.h"

#include <iostream>

using namespace cv;

int main()
{
  Mat R(3,3,CV_32FC1);
  
  for(unsigned int i = 0; i < 3; i++)
    {
    for(unsigned int j = 0; j < 3; j++)
      {
      R.at<float>(i,j) = 2;
      }
    }
  

  for(unsigned int i = 0; i < 3; i++)
    {
    for(unsigned int j = 0; j < 3; j++)
      {
      std::cout << R.at<float>(i,j) << std::endl;
      }
    }
     
  return 0;
}

