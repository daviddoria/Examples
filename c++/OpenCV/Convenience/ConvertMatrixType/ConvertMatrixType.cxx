#include "cv.h"

#include <iostream>

cv::Mat GetFloat(cv::Mat M);

int main(int, char*[])
{
  cv::Mat doubleMatrix(3,3,CV_64FC1);

  for(unsigned int i = 0; i < 3; i++)
    {
    for(unsigned int j = 0; j < 3; j++)
      {
      doubleMatrix.at<float>(i,j) = 1;
      }
    }

  cv::Mat floatMatrix(3,3,CV_32FC1);
  floatMatrix = GetFloat(doubleMatrix);

  return 0;
}


cv::Mat GetFloat(cv::Mat M)
{
  // This function converts a matrix from double to float
  if(M.depth() == 5)
  {
    return M;
  }

  if(M.depth() == 6)
  {
    cv::Mat K(M.rows,M.cols,CV_32FC1);
    for(unsigned int row = 0; row < 3; row++)
    {
      for(unsigned int col = 0; col < 3; col++)
      {
      K.at<float>(row,col) = M.at<double>(row,col);
      }
    }
    return K;
  }
  else
  {
    std::cerr << "Check the type of M! It is depth " << M.depth() << " !" << std::endl;
    exit(-1);
  }
}