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
  std::string image1FileName = argv[1];
  std::string image2FileName = argv[2];
  std::string points1FileName = argv[3];
  std::string points2FileName = argv[4];

  // Output arguments
  std::cout << "Image1: " << image1FileName << std::endl
            << "Image2: " << image2FileName << std::endl
            << "Points1: " << points1FileName << std::endl
            << "Points2: " << points2FileName << std::endl;
  
  // Read images
  cv::Mat image1 = cv::imread(image1FileName, 1);
  cv::Mat image2 = cv::imread(image2FileName, 1);

  // Read points
  std::vector<cv::Point2f> points1 = ReadPoints(points1FileName);
  std::vector<cv::Point2f> points2 = ReadPoints(points2FileName);
  
  if(points1.size() != points2.size())
    {
    std::cerr << "There must be the same number of points in both files (since they are correspondences!). File1 has " << points1.size() << " while file2 has " << points2.size() << std::endl;
    return -1;
    }

  cv::Mat fundamentalMatrix = cv::findFundamentalMat(points1, points2, cv::FM_RANSAC, 3, 0.99);

  std::cout << "F = " << fundamentalMatrix << std::endl;
  
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
