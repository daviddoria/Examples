#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <string>

int main( int argc, char* argv[])
{
  cv::Mat K(3,3,CV_64F);

  cv::Size imageSize(100,200);
  double apertureWidth = 1;
  double apertureHeight = 1;
  double fieldOfViewX;
  double fieldOfViewY;
  double focalLength;
  cv::Point2d principalPoint;
  double aspectRatio;
  cv::calibrationMatrixValues(K, imageSize, apertureWidth, apertureHeight, fieldOfViewX, fieldOfViewY, focalLength, principalPoint, aspectRatio);

  return 0;
}
