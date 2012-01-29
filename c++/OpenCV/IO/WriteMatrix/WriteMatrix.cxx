#include "cv.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"

#include <iostream>

int main(int argc, char*argv[])
{
  // Create a matrix
  cv::Mat myMatrix(3,3,CV_32FC1);
  std::cout << "Input:" << std::endl;
  for(unsigned int i = 0; i < 3; i++)
    {
    for(unsigned int j = 0; j < 3; j++)
      {
      myMatrix.at<float>(i,j) = 2.1;
      }
    }

  std::cout << myMatrix << std::endl;

  // Write the matrix to a file
  cv::FileStorage fsout("test.yml", cv::FileStorage::WRITE);
  fsout << "myMatrix" << myMatrix;

  // Read the file back in to test that it was written correctly
  cv::Mat myMatrix2(3,3,CV_32FC1);
  cv::FileStorage fsin("test.yml", cv::FileStorage::READ);
  fsin["myMatrix"] >> myMatrix2;

  std::cout << myMatrix2 << std::endl;

  return 0;
}

