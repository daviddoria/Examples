#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <string>

using namespace cv;

std::vector<Point3f> Create3DChessboardCorners(Size boardSize, float squareSize);

int main( int argc, char* argv[])
{
  Size boardSize(7,7); // the number of "inside corners" (where a black square meets a white square). This board is actually 8x8 squares

  float squareSize = 1.f; // This is "1 arbitrary unit"

  std::string imageFileName = argv[1];

  Mat image = imread(imageFileName, 1);

  namedWindow( "Image View", 1 );

  Size imageSize = image.size();

  // Find the chessboard corners
  vector<vector<Point2f> > imagePoints(1);
  bool found = findChessboardCorners(image, boardSize, imagePoints[0]);
  if(!found)
    {
    std::cerr << "Could not find chess board!" << std::endl;
    exit(-1);
    }

  drawChessboardCorners(image, boardSize, Mat(imagePoints[0]), found );

  std::vector<std::vector<Point3f> > objectPoints(1);
  objectPoints[0] = Create3DChessboardCorners(boardSize, squareSize);

  std::vector<Mat> rotationVectors;
  std::vector<Mat> translationVectors;

  Mat distortionCoefficients = Mat::zeros(8, 1, CV_64F); // There are 8 distortion coefficients
  Mat cameraMatrix = Mat::eye(3, 3, CV_64F);

  int flags = 0;
  double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                  distortionCoefficients, rotationVectors, translationVectors, flags|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);

  std::cout << "RMS: " << rms << std::endl;

  std::cout << "Ccamera matrix: " << cameraMatrix << std::endl;
  std::cout << "Distortion _coefficients: " << distortionCoefficients << std::endl;

  imshow("Image View", image);
  waitKey(0);

  return 0;
}

std::vector<Point3f> Create3DChessboardCorners(Size boardSize, float squareSize)
{
  // This function creates the 3D points of your chessboard in its own coordinate system
  
  std::vector<Point3f> corners;

  for( int i = 0; i < boardSize.height; i++ )
  {
    for( int j = 0; j < boardSize.width; j++ )
    {
      corners.push_back(Point3f(float(j*squareSize),
                                float(i*squareSize), 0));
    }
  }

  return corners;
}