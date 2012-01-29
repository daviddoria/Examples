#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <fstream>
#include <iostream>
#include <string>

std::vector<cv::Point2f> Read2DPoints(const std::string& filename);
std::vector<cv::Point3f> Read3DPoints(const std::string& filename);

int main( int argc, char* argv[])
{
  // Verify arguments
  if(argc < 4)
    {
    std::cerr << "Required: image.png imagePoints.txt objectPoints.txt" << std::endl;
    return -1;
    }

  // Parse arguments
  std::string imageFileName = argv[1];
  std::string imagePointsFileName = argv[2];
  std::string objectPointsFileName = argv[3];

  // Output arguments
  std::cout << "imageFileName: " << imageFileName << std::endl;
  std::cout << "imagePointsFileName: " << imagePointsFileName << std::endl;
  std::cout << "objectPointsFileName: " << objectPointsFileName << std::endl;

  // Read image
  cv::Mat image = cv::imread(imageFileName.c_str(), CV_LOAD_IMAGE_COLOR);

  // Read points
  std::vector<cv::Point2f> inputImagePoints = Read2DPoints(imagePointsFileName);
  std::vector<cv::Point3f> inputObjectPoints = Read3DPoints(objectPointsFileName);

  // Since we only have one view, set the point sets as the first element of the vectors
  std::vector<std::vector<cv::Point2f> > imagePoints;
  imagePoints.push_back(inputImagePoints);
  
  std::vector<std::vector<cv::Point3f> > objectPoints;
  objectPoints.push_back(inputObjectPoints);

  cv::Mat cameraMatrix;
  cv::Mat distCoeffs;
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
  cv::calibrateCamera(objectPoints, imagePoints, image.size(), cameraMatrix, distCoeffs, rvecs, tvecs);
  
  return 0;
}


std::vector<cv::Point2f> Read2DPoints(const std::string& filename)
{
  // Read points
  std::ifstream pointsstream(filename.c_str());

  if(pointsstream == NULL)
    {
    std::cout << "Cannot open file " << filename << std::endl;
    exit(-1);
    }

  // Read the point from the first image
  std::string line;
  std::vector<cv::Point2f> points;

  while(getline(pointsstream, line))
  {
    std::stringstream ss(line);
    float x,y;
    ss >> x >> y;
    points.push_back(cv::Point2f(x,y));
  }

  return points;
}


std::vector<cv::Point3f> Read3DPoints(const std::string& filename)
{
  // Read points
  std::ifstream pointsstream(filename.c_str());

  if(pointsstream == NULL)
    {
    std::cout << "Cannot open file " << filename << std::endl;
    exit(-1);
    }

  // Read the point from the first image
  std::string line;
  std::vector<cv::Point3f> points;

  while(getline(pointsstream, line))
  {
    std::stringstream ss(line);
    float x,y,z;
    ss >> x >> y >> z;
    points.push_back(cv::Point3f(x,y,z));
  }

  return points;
}
