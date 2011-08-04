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
  cv::Mat image1 = cv::imread(image1FileName, CV_LOAD_IMAGE_COLOR);
  cv::Mat image2 = cv::imread(image2FileName, CV_LOAD_IMAGE_COLOR);

  std::cout << "image1 has " << image1.channels() << " channels." << std::endl;
  std::cout << "image1 has type " << image1.type() << std::endl;
  std::cout << "image1 has " << image1.cols << " columns." << std::endl;
  std::cout << "image1 has " << image1.rows << " rows." << std::endl;

  // Read points
  std::vector<cv::Point2f> points1 = ReadPoints(points1FileName);
  std::vector<cv::Point2f> points2 = ReadPoints(points2FileName);

  if(points1.size() != points2.size())
    {
    std::cerr << "There must be the same number of points in both files (since they are correspondences!). File1 has " << points1.size() << " while file2 has " << points2.size() << std::endl;
    return -1;
    }
/*
  cv::Mat fundamentalMatrix = cv::findFundamentalMat(points1, points2, cv::FM_RANSAC, 3, 0.99);

  std::cout << "F = " << fundamentalMatrix << std::endl;

  cv::Mat H1, H2;
  cv::stereoRectifyUncalibrated(points1, points2, fundamentalMatrix, image2.size(), H1, H2);

  std::cout << "H1 = " << H1 << std::endl;
  std::cout << "H2 = " << H2 << std::endl;

  cv::Mat H = cv::findHomography(points1, points2);

  std::cout << "H: " << H << std::endl;

  cv::Mat rectified(3,3,CV_LOAD_IMAGE_COLOR);
  cv::perspectiveTransform(image2, rectified, H2);

  cv::imwrite("output.png", rectified);
  */

  // We really just want to pass the vectors 'points1' and 'points2', but the function expects a c-style array, so we pass the address of the first element
  cv::Mat P = cv::getPerspectiveTransform(&points1[0], &points2[0]);

  std::cout << "P = " << P << std::endl;

  cv::Mat rectified = image1;
  cv::warpPerspective(image1, rectified, P, image1.size());
  /*
  //cv::Mat rectified(image1.cols, image1.rows, CV_32FC3);
  // Create an image the same size as the input
  cv::Mat rectified = image1;

  cv::Mat P4(4,4, P.type());
  std::cout << P4 << std::endl;

  for(unsigned int i = 0; i < 3; i++)
    {
    for(unsigned int j = 0; j < 3; j++)
      {
      P4.at<float>(i,j) = P.at<float>(i,j);
      }
    }
  P4.at<float>(3,3) = 1;
  std::cout << "Final P4: " << P4 << std::endl;

  //cv::perspectiveTransform(image1, rectified, P);
  cv::perspectiveTransform(image1, rectified, P4);

  std::cout << "rectified channels after: " << rectified.channels() << std::endl;
  */

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
    std::stringstream ss(line);
    float x,y;
    ss >> x >> y;
    points.push_back(cv::Point2f(x,y));
  }

  return points;
}
