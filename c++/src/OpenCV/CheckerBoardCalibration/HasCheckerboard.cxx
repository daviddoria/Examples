#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <string>

std::vector<cv::Point3f> Create3DChessboardCorners(cv::Size boardSize, float squareSize);

int main( int argc, char* argv[])
{
  if(argc != 3)
    {
    std::cerr << "Required inputs: imageFilename, NumberOfSquaresPerDimension" << std::endl;
    exit(-1);
    }
    
  std::string imageFileName = argv[1];
  std::string strSquares = argv[2];
  std::stringstream ss;
  ss << strSquares;
  int numberOfSquares;
  ss >> numberOfSquares;
  
  // Specify the number of squares along each dimension of the board.
  // This is actually the number of "inside corners" (where a black square meets a white square).
  // That is, if the board is composed of n x m squares, you would use (n-1, m-1) as the arguments.
  // For example, for a standard checkerboard (8x8 squares), you would use:
  cv::Size boardSize(numberOfSquares-1, numberOfSquares-1);

  float squareSize = 1.f; // This is "1 arbitrary unit"
  
  std::cout << "Reading " << imageFileName << std::endl;

  cv::Mat image = cv::imread(imageFileName, 1);
  if(image.empty())
    {
    std::cerr << "Image not read correctly!" << std::endl;
    exit(-1);
    }

  cv::Size imageSize = image.size();

  // Find the chessboard corners
  std::vector<std::vector<cv::Point2f> > imagePoints(1);
  bool found = findChessboardCorners(image, boardSize, imagePoints[0]);
  if(!found)
    {
    std::cerr << "Could not find checkerboard board!" << std::endl;
    exit(-1);
    }
  else
    {
    std::cout << "Found checkerboard!" << std::endl;
    }

  return 0;
}

std::vector<cv::Point3f> Create3DChessboardCorners(cv::Size boardSize, float squareSize)
{
  // This function creates the 3D points of your chessboard in its own coordinate system

  std::vector<cv::Point3f> corners;

  for( int i = 0; i < boardSize.height; i++ )
  {
    for( int j = 0; j < boardSize.width; j++ )
    {
      corners.push_back(cv::Point3f(float(j*squareSize),
                                float(i*squareSize), 0));
    }
  }

  return corners;
}