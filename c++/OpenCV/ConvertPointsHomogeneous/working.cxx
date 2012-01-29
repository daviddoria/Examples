#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <string>

void UseDoubles();
void UseFloats();

void MultiChannel();

int main( int argc, char* argv[])
{
  //UseDoubles();
  //UseFloats();
  MultiChannel();
  return 0;
}

void MultiChannel()
{
  float v[] = { 1, 2, 3, 4 };
  float w[3];
  std::cout << "v: " << v[0] << " " << v[1] << " " << v[2] << " " << v[3] << std::endl;

  cv::Mat V(1, 1, CV_32FC4, v);
  std::cout << "V: " << V.at<float>(0,0) << " " << V.at<float>(0,1) << " " << V.at<float>(0,2) << " " << V.at<float>(0,3) << std::endl;

  cv::Mat W(1, 1, CV_32FC3, w);
  cv::convertPointsFromHomogeneous(V, W);
  std::cout << "W: " << W.at<float>(0,0) << " " << W.at<float>(0,1) << " " << W.at<float>(0,2) << std::endl;
  std::cout << "w: " << w[0] << " " << w[1] << " " << w[2] << std::endl;
  //printf("%f %f %f\n", w[0], w[1], w[2]);
}

void UseDoubles()
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
  //cv::convertPointsHomogeneous(Thomogeneous, T);
  cv::convertPointsFromHomogeneous(Thomogeneous, T);

  std::cout << "T: " << T << std::endl;
}

void UseFloats()
{
  // Create the known projection matrix
  cv::Mat Thomogeneous(1,4,cv::DataType<float>::type);// This returns 1?
  //cv::Mat Thomogeneous(4,1,cv::DataType<float>::type);// This returns -1?
  //cv::Mat Thomogeneous(4,1,CV_32FC1);// This returns -1?
  //cv::Mat Thomogeneous(1,4,CV_32FC1); // This returns 1?

  std::cout << "Thomogeneous.depth() = " << Thomogeneous.depth() << std::endl;
  std::cout << "Thomogeneous.checkVector(3) = " << Thomogeneous.checkVector(3) << std::endl;
  std::cout << "Thomogeneous.checkVector(4) = " << Thomogeneous.checkVector(4) << std::endl;

  std::cout << "Before setting values" << std::endl;
  std::cout << "Thomogeneous.channels() = " << Thomogeneous.channels() << std::endl;
  std::cout << "Thomogeneous.rows() = " << Thomogeneous.rows << std::endl;
  std::cout << "Thomogeneous.cols() = " << Thomogeneous.cols << std::endl;
  /*
  Thomogeneous.at<float>(0,0) = -2.8058e-01;
  Thomogeneous.at<float>(1,0) = -6.8326e-02;
  Thomogeneous.at<float>(2,0) = 5.1458e-07;
  Thomogeneous.at<float>(3,0) = 5.1458e-07;
  */

  Thomogeneous.at<float>(0,0) = -2.8058e-01;
  Thomogeneous.at<float>(0,1) = -6.8326e-02;
  Thomogeneous.at<float>(0,2) = 5.1458e-07;
  Thomogeneous.at<float>(0,3) = 5.1458e-07;

  std::cout << "After setting values" << std::endl;
  std::cout << "Thomogeneous.channels() = " << Thomogeneous.channels() << std::endl;
  std::cout << "Thomogeneous.rows() = " << Thomogeneous.rows << std::endl;
  std::cout << "Thomogeneous.cols() = " << Thomogeneous.cols << std::endl;

  std::cout << "Thomogeneous: " << Thomogeneous << std::endl;

  //cv::Mat T(3,1,cv::DataType<double>::type); // translation vector
  cv::Mat T;
  //cv::convertPointsHomogeneous(Thomogeneous, T);
  cv::convertPointsFromHomogeneous(Thomogeneous, T);

  std::cout << "T: " << T << std::endl;
}
