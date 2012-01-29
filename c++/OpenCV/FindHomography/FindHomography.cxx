#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <vector>
#include <string>
#include <fstream>
#include <iostream>

std::vector<cv::Point2f> ReadPoints(const std::string& filename);

int main(int argc, char* argv[])
{
  // Verify arguments
  if(argc < 5)
    {
    std::cerr << "Required arguments: source.jpg destination.jpg source.txt destination.txt" << std::endl;
    return -1;
    }

  // Parse arguments
  std::string sourceImageFileName = argv[1];
  std::string destinationImageFileName = argv[2];
  std::string sourcePointsFileName = argv[3];
  std::string destinationPointsFileName = argv[4];

  // Output arguments
  std::cout << "sourceImageFileName: " << sourceImageFileName << std::endl
            << "destinationImageFileName: " << destinationImageFileName << std::endl
            << "sourcePointsFileName: " << sourcePointsFileName << std::endl
            << "destinationPointsFileName: " << destinationPointsFileName << std::endl;
  
  // Read images
  cv::Mat sourceImage = cv::imread(sourceImageFileName, 1);
  cv::Mat destinationImage = cv::imread(destinationImageFileName, 1);

  // Read points
  std::vector<cv::Point2f> sourcePoints = ReadPoints(sourcePointsFileName);
  std::vector<cv::Point2f> destinationPoints = ReadPoints(destinationPointsFileName);
  
  if(sourcePoints.size() != destinationPoints.size())
    {
    std::cerr << "There must be the same number of points in both files (since they are correspondences!)."
	      << "Source has " << sourcePoints.size() << " while destinationPoints has " << destinationPoints.size() << std::endl;
    return -1;
    }

  //cv::Mat H = cv::findHomography(sourcePoints, destinationPoints);
  //cv::Mat H = cv::findHomography(sourcePoints, destinationPoints, CV_RANSAC);
  cv::Mat H = cv::findHomography(sourcePoints, destinationPoints, CV_LMEDS);
  
  cv::Mat rectified(sourceImage.size(), sourceImage.type());
  cv::warpPerspective(sourceImage, rectified, H, sourceImage.size());
  cv::imwrite("rectified_lmeds.jpg", rectified);
  
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
    std::stringstream ss(line);
    float x,y;
    ss >> x >> y;
    points.push_back(cv::Point2f(x,y));
  }

  return points;
}
