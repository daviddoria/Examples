#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
 
#include <iostream>
#include <string>
 
std::vector<cv::Point2f> Generate2DPoints();
std::vector<cv::Point3f> Generate3DPoints();
 
int main( int argc, char* argv[])
{
  // Read points
  std::vector<cv::Point2f> imagePoints = Generate2DPoints();
  std::vector<cv::Point3f> objectPoints = Generate3DPoints();
 
  std::cout << "There are " << imagePoints.size() << " imagePoints and " << objectPoints.size() << " objectPoints." << std::endl;
  cv::Mat cameraMatrix(3,3,cv::DataType<double>::type);
  cv::setIdentity(cameraMatrix);
 
  std::cout << "Initial cameraMatrix: " << cameraMatrix << std::endl;
 
  cv::Mat distCoeffs(4,1,cv::DataType<double>::type);
  distCoeffs.at<double>(0) = 0;
  distCoeffs.at<double>(1) = 0;
  distCoeffs.at<double>(2) = 0;
  distCoeffs.at<double>(3) = 0;
 
  cv::Mat rvec(3,1,cv::DataType<double>::type);
  cv::Mat tvec(3,1,cv::DataType<double>::type);
 
  cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
 
  std::cout << "rvec: " << rvec << std::endl;
  std::cout << "tvec: " << tvec << std::endl;
 
  std::vector<cv::Point2f> projectedPoints;
  cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);
 
  for(unsigned int i = 0; i < projectedPoints.size(); ++i)
    {
    std::cout << "Image point: " << imagePoints[i] << " Projected to " << projectedPoints[i] << std::endl;
    }
 
  return 0;
}
 
 
std::vector<cv::Point2f> Generate2DPoints()
{
  std::vector<cv::Point2f> points;
 
  float x,y;
 
  x=282;y=274;
  points.push_back(cv::Point2f(x,y));
 
  x=397;y=227;
  points.push_back(cv::Point2f(x,y));
 
  x=577;y=271;
  points.push_back(cv::Point2f(x,y));
 
  x=462;y=318;
  points.push_back(cv::Point2f(x,y));
 
  x=270;y=479;
  points.push_back(cv::Point2f(x,y));
 
  x=450;y=523;
  points.push_back(cv::Point2f(x,y));
 
  x=566;y=475;
  points.push_back(cv::Point2f(x,y));
 
  for(unsigned int i = 0; i < points.size(); ++i)
    {
    std::cout << points[i] << std::endl;
    }
 
  return points;
}
 
 
std::vector<cv::Point3f> Generate3DPoints()
{
  std::vector<cv::Point3f> points;
 
 
  float x,y,z;
 
  x=.5;y=.5;z=-.5;
  points.push_back(cv::Point3f(x,y,z));
 
  x=.5;y=.5;z=.5;
  points.push_back(cv::Point3f(x,y,z));
 
  x=-.5;y=.5;z=.5;
  points.push_back(cv::Point3f(x,y,z));
 
  x=-.5;y=.5;z=-.5;
  points.push_back(cv::Point3f(x,y,z));
 
  x=.5;y=-.5;z=-.5;
  points.push_back(cv::Point3f(x,y,z));
 
  x=-.5;y=-.5;z=-.5;
  points.push_back(cv::Point3f(x,y,z));
 
  x=-.5;y=-.5;z=.5;
  points.push_back(cv::Point3f(x,y,z));
 
  for(unsigned int i = 0; i < points.size(); ++i)
    {
    std::cout << points[i] << std::endl;
    }
 
  return points;
}
