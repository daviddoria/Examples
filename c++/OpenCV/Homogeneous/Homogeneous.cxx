#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <string>

int main( int argc, char* argv[])
{
  {
  // From a 3 vector to a 2 vector (divide by 3rd coordinate)
  // error:src.checkVector(3) >= 0 && (src.depth() == CV_32F || src.depth() == CV_32S) in function convertPointsHomogeneous

  cv::Mat point(3,1,CV_32FC1);
  point.at<float>(0,0) = 1;
  point.at<float>(1,0) = 2;
  point.at<float>(2,0) = 3;
  std::cout << point << std::endl;

  std::vector<cv::Point2f> homogeneous(1);
  cv::convertPointsHomogeneous(point, homogeneous);
  std::cout << homogeneous[0] << std::endl;
  }
/*
  {
  // From a 3 vector to a 4 vector (add a 1)
  cv::Mat point(3,1,CV_32FC1);
  point.at<float>(0,0) = 1;
  point.at<float>(1,0) = 2;
  point.at<float>(2,0) = 3;
  std::cout << point << std::endl;

  std::vector<cv::Point4f> homogeneous(1); // Point4f is not a class!
  cv::convertPointsHomogeneous(point, homogeneous);
  std::cout << homogeneous[0] << std::endl;
  }
  */
  return 0;
}
