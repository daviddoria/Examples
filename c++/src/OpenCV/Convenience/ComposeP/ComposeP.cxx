#include "cv.h"

#include <iostream>

cv::Mat ConstructP(cv::Mat R, cv::Mat T, cv::Mat K);

int main(int, char*[])
{
  // Generate test R, K, and T
  cv::Mat R(3,3,CV_32FC1);

  for(unsigned int i = 0; i < 3; i++)
    {
    for(unsigned int j = 0; j < 3; j++)
      {
      R.at<float>(i,j) = 1;
      }
    }
  std::cout << "R: " << R << std::endl;

  cv::Mat K(3,3,CV_32FC1);
  for(unsigned int i = 0; i < 3; i++)
    {
    for(unsigned int j = 0; j < 3; j++)
      {
      K.at<float>(i,j) = 2;
      }
    }
  std::cout << "K: " << K << std::endl;

  cv::Mat T(3,1,CV_32FC1);
  for(unsigned int i = 0; i < 3; i++)
    {
    T.at<float>(i,0) = 3;
    }
  std::cout << "T: " << T << std::endl;

  cv::Mat P = ConstructP(R, T, K);
  std::cout << "P: " << P << std::endl;

  return 0;
}


cv::Mat ConstructP(cv::Mat R, cv::Mat T, cv::Mat K)
{
  cv::Mat P(3,4,CV_32FC1);
  for(unsigned int i = 0; i < 3; i++)
    {
    for(unsigned int j = 0; j < 3; j++)
      {
      P.at<float>(i,j) = R.at<float>(i,j);
      }
    }

  for(unsigned int row = 0; row < 3; row++)
    {
    P.at<float>(row,3) = T.at<float>(row,0);
    }


  P = K * P;

  return P;
}