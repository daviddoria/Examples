#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

std::vector<cv::Point2f> ReadPoints(const std::string& filename);

int main( int argc, char* argv[])
{
  // Verify arguments
  if(argc < 5)
    {
    std::cerr << "Required arguments: input1.jpg input2.jpg points1.txt points2.txt" << std::endl;
    return -1;
    }

  // Parse arguments
  std::string sourceImageFileName = argv[1];
  std::string destinationImageFileName = argv[2];
  std::string sourcePointsFileName = argv[3];
  std::string destinationPointsFileName = argv[4];

  // Output arguments
  std::cout << "Source image: " << sourceImageFileName << std::endl
            << "Destination image: " << destinationImageFileName << std::endl
            << "Points1: " << sourcePointsFileName << std::endl
            << "Points2: " << destinationPointsFileName << std::endl;

  // Read images
  cv::Mat sourceImage = cv::imread(sourceImageFileName, CV_LOAD_IMAGE_COLOR);
  cv::Mat destinationImage = cv::imread(destinationImageFileName, CV_LOAD_IMAGE_COLOR);
/*
  std::cout << "image1 has " << image1.channels() << " channels." << std::endl;
  std::cout << "image1 has type " << image1.type() << std::endl;
  std::cout << "image1 has " << image1.cols << " columns." << std::endl;
  std::cout << "image1 has " << image1.rows << " rows." << std::endl;
*/
  // Read points
  std::vector<cv::Point2f> sourcePoints = ReadPoints(sourcePointsFileName);
  std::vector<cv::Point2f> destinationPoints = ReadPoints(destinationPointsFileName);

  if(sourcePoints.size() != destinationPoints.size())
    {
    std::cerr << "There must be the same number of points in both files (since they are correspondences!). Source poitns has " << sourcePoints.size() << " while destination poitns  has " << destinationPoints.size() << std::endl;
    return -1;
    }

  // We really just want to pass the vectors 'points1' and 'points2', but the function expects a c-style array, so we pass the address of the first element
  cv::Mat P = cv::getPerspectiveTransform(&sourcePoints[0], &destinationPoints[0]);

  std::cout << "P = " << P << std::endl;

  cv::Mat rectified(sourceImage.size(), sourceImage.type());
  cv::warpPerspective(sourceImage, rectified, P, sourceImage.size());
  cv::imwrite("output.png", rectified);
  
  return 0;
}

std::vector<cv::Point2f> ReadPoints(const std::string& filename)
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
    if(points.size() == 4)
    {
      return points;
    }
    
    std::stringstream ss(line);
    float x,y;
    ss >> x >> y;
    points.push_back(cv::Point2f(x,y));
  }

  return points;
}
