#include <cv.h>
#include <highgui.h>

#include <iostream>

int main(int argc, char*argv[])
{
  // Create a synthetic projection matrix
  cv::Mat P(3,4,cv::DataType<float>::type);
  P.at<float>(0,0) = 1;
  P.at<float>(1,0) = 0;
  P.at<float>(2,0) = 0;

  P.at<float>(0,1) = 0;
  P.at<float>(1,1) = 1;
  P.at<float>(2,1) = 0;

  P.at<float>(0,2) = 5;
  P.at<float>(1,2) = 4;
  P.at<float>(2,2) = 1;

  P.at<float>(0,3) = 1;
  P.at<float>(1,3) = 2;
  P.at<float>(2,3) = 3;

  std::cout << "P: " << P << std::endl;

  // Decompose the projection matrix into:
  cv::Mat K(3,3,cv::DataType<float>::type); // intrinsic parameter matrix
  cv::Mat R(3,3,cv::DataType<float>::type); // rotation matrix
  cv::Mat T(4,1,cv::DataType<float>::type); // translation vector

  cv::decomposeProjectionMatrix(P, K, R, T);
  std::cout << "K: " << K << std::endl;
  std::cout << "R: " << R << std::endl;
  std::cout << "T: " << T << std::endl;

  return 0;
}
