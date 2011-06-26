#include "cv.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <string>

std::vector<cv::Point3f> Create3DChessboardCorners(cv::Size boardSize, float squareSize);

struct CameraStruct
{
  cv::Mat CameraMatrix;
  cv::Mat ImagePoints;
  cv::Mat DistortionCoefficients;
  cv::Mat Image;
};

CameraStruct CalibrateCamera(cv::Mat image, cv::Size boardSize);

int main( int argc, char* argv[])
{
  std::string imageFileName = argv[1];
  std::string outputFileName = argv[2];

  cv::Mat image = cv::imread(imageFileName, 1);

  if(image.empty())
    {
    std::cerr << "Could not read image!" << std::endl;
    return -1;
    }

  CameraStruct camera = CalibrateCamera(image, cv::Size(6,9)); // If the board does not have the name number of squares in each dimension, the order of boardSize is not important.

  cv::Mat outputImage;
  cv::undistort(camera.Image, outputImage, camera.CameraMatrix, camera.DistortionCoefficients);

  cv::imwrite(outputFileName, outputImage);

  return 0;
}

CameraStruct CalibrateCamera(cv::Mat image, cv::Size boardSize)
{
  float squareSize = 1.f; // This is "1 arbitrary unit"

  cv::Size imageSize = image.size();

  // Find the chessboard corners
  std::vector<std::vector<cv::Point2f> > imagePoints(1);
  bool found = findChessboardCorners(image, boardSize, imagePoints[0]);
  if(!found)
    {
    std::cerr << "Could not find chess board!" << std::endl;
    exit(-1);
    }

  drawChessboardCorners(image, boardSize, cv::Mat(imagePoints[0]), found );

  std::vector<std::vector<cv::Point3f> > objectPoints(1);
  objectPoints[0] = Create3DChessboardCorners(boardSize, squareSize);

  std::vector<cv::Mat> rotationVectors;
  std::vector<cv::Mat> translationVectors;

  cv::Mat distortionCoefficients = cv::Mat::zeros(8, 1, CV_64F); // There are 8 distortion coefficients
  cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);

  int flags = 0;
  double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                  distortionCoefficients, rotationVectors, translationVectors, flags|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);

  CameraStruct camera;
  camera.CameraMatrix = cameraMatrix;
  camera.ImagePoints = cv::Mat(imagePoints[0]);
  camera.DistortionCoefficients = distortionCoefficients;
  camera.Image = image;

  return camera;
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