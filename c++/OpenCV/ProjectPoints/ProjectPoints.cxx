#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <string>

std::vector<cv::Point2d> Generate2DPoints();
std::vector<cv::Point3d> Generate3DPoints();

int main( int argc, char* argv[])
{
  // Read points
  std::vector<cv::Point2d> imagePoints = Generate2DPoints();
  std::vector<cv::Point3d> objectPoints = Generate3DPoints();

  // Create the known projection matrix
  cv::Mat P(3,4,cv::DataType<double>::type);
  P.at<double>(0,0) = -2.8058e-01;
  P.at<double>(1,0) = -6.8326e-02;
  P.at<double>(2,0) = 5.1458e-07;

  P.at<double>(0,1) = 2.0045e-02;
  P.at<double>(1,1) = -3.1718e-01;
  P.at<double>(2,1) = 4.5840e-06;

  P.at<double>(0,2) = 1.8102e-01;
  P.at<double>(1,2) = -7.2974e-02;
  P.at<double>(2,2) = 2.6699e-06;

  P.at<double>(0,3) = 6.6062e-01;
  P.at<double>(1,3) = 5.8402e-01;
  P.at<double>(2,3) = 1.5590e-03;

    // Decompose the projection matrix into:
  cv::Mat K(3,3,cv::DataType<double>::type); // intrinsic parameter matrix
  cv::Mat rvec(3,3,cv::DataType<double>::type); // rotation matrix
  
  cv::Mat Thomogeneous(4,1,cv::DataType<double>::type); // translation vector

  cv::decomposeProjectionMatrix(P, K, rvec, Thomogeneous);

  cv::Mat T(3,1,cv::DataType<double>::type); // translation vector
  //cv::Mat T;
  cv::convertPointsHomogeneous(Thomogeneous, T);

  std::cout << "K: " << K << std::endl;
  std::cout << "rvec: " << rvec << std::endl;
  std::cout << "T: " << T << std::endl;
  
  // Create zero distortion
  cv::Mat distCoeffs(4,1,cv::DataType<double>::type);
  distCoeffs.at<double>(0) = 0;
  distCoeffs.at<double>(1) = 0;
  distCoeffs.at<double>(2) = 0;
  distCoeffs.at<double>(3) = 0;

  std::vector<cv::Point2f> projectedPoints;

  cv::Mat rvecR(3,1,cv::DataType<double>::type);//rodrigues rotation matrix
  cv::Rodrigues(rvec,rvecR);
  
  cv::projectPoints(objectPoints, rvecR, T, K, distCoeffs, projectedPoints);

  for(unsigned int i = 0; i < projectedPoints.size(); ++i)
    {
    std::cout << "Image point: " << imagePoints[i] << " Projected to " << projectedPoints[i] << std::endl;
    }

  return 0;
}


std::vector<cv::Point2d> Generate2DPoints()
{
  std::vector<cv::Point2d> points;

  double x,y;

  x=282;y=274;
  points.push_back(cv::Point2d(x,y));

  x=397;y=227;
  points.push_back(cv::Point2d(x,y));

  x=577;y=271;
  points.push_back(cv::Point2d(x,y));

  x=462;y=318;
  points.push_back(cv::Point2d(x,y));

  x=270;y=479;
  points.push_back(cv::Point2d(x,y));

  x=450;y=523;
  points.push_back(cv::Point2d(x,y));

  x=566;y=475;
  points.push_back(cv::Point2d(x,y));
  /*
  for(unsigned int i = 0; i < points.size(); ++i)
    {
    std::cout << points[i] << std::endl;
    }
  */
  return points;
}


std::vector<cv::Point3d> Generate3DPoints()
{
  std::vector<cv::Point3d> points;

  double x,y,z;

  x=.5;y=.5;z=-.5;
  points.push_back(cv::Point3d(x,y,z));

  x=.5;y=.5;z=.5;
  points.push_back(cv::Point3d(x,y,z));

  x=-.5;y=.5;z=.5;
  points.push_back(cv::Point3d(x,y,z));

  x=-.5;y=.5;z=-.5;
  points.push_back(cv::Point3d(x,y,z));

  x=.5;y=-.5;z=-.5;
  points.push_back(cv::Point3d(x,y,z));

  x=-.5;y=-.5;z=-.5;
  points.push_back(cv::Point3d(x,y,z));

  x=-.5;y=-.5;z=.5;
  points.push_back(cv::Point3d(x,y,z));

  /*
  for(unsigned int i = 0; i < points.size(); ++i)
    {
    std::cout << points[i] << std::endl;
    }
  */
  return points;
}
