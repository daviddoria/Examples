#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <string>

int main( int argc, char* argv[])
{
  // Create the known projection matrix
  cv::Mat Thomogeneous(4,1,cv::DataType<double>::type);
  Thomogeneous.at<double>(0,0) = -2.8058e-01;
  Thomogeneous.at<double>(1,0) = -6.8326e-02;
  Thomogeneous.at<double>(2,0) = 5.1458e-07;
  Thomogeneous.at<double>(3,0) = 5.1458e-07;

  std::cout << "Thomogeneous: " << Thomogeneous << std::endl;
  
  //cv::Mat T(3,1,cv::DataType<double>::type); // translation vector
  cv::Mat T;
  cv::convertPointsHomogeneous(Thomogeneous, T);

  std::cout << "T: " << T << std::endl;

  return 0;
}
