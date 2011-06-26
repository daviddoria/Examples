/* NOTE:
You need at least two pairs from the same two cameras.
*/

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

using namespace cv;

Mat GetFloat(Mat M);

Mat ConstructP(Mat R, Mat T, Mat K);

std::vector<Point3f> Create3DChessboardCorners(Size boardSize, float squareSize);

int main(int argc, char* argv[])
{
  //Size boardSize(7,7);
  Size boardSize(6,9);

  std::string camera1image1FileName = argv[1];
  std::string camera1image2FileName = argv[2];

  std::string camera2image1FileName = argv[3];
  std::string camera2image2FileName = argv[4];

  const float squareSize = 1.f;

  bool color = false;
  Mat camera1image1 = imread(camera1image1FileName, color);
  Mat camera1image2 = imread(camera1image2FileName, color);

  std::cout << "camera1image1 depth: " << camera1image1.depth() << std::endl;
  
  Mat camera2image1 = imread(camera2image1FileName, color);
  Mat camera2image2 = imread(camera2image2FileName, color);

  /*
  Mat camera1image1 = imread(camera1image1FileName, 1); // 1 is color, 0 is grayscale
  Mat camera1image2 = imread(camera1image2FileName, 1);

  Mat camera2image1 = imread(camera2image1FileName, 1);
  Mat camera2image2 = imread(camera2image2FileName, 1);
  */

  Size imageSize = camera1image1.size();

  if(camera1image2.size() != imageSize || camera1image1.size() != imageSize || camera2image2.size() != imageSize )
  {
    std::cerr << "All images must be the same size!" << std::endl;
    return -1;
  }

  // find corners of the first board
  std::vector<std::vector<Point2f> > camera1ImagePoints(2);
  bool found1_1 = findChessboardCorners(camera1image1, boardSize, camera1ImagePoints[0]);
  if(!found1_1)
    {
    std::cerr << "Checkboard1_1 corners not found!" << std::endl;
    exit(-1);
    }

  bool found1_2 = findChessboardCorners(camera1image2, boardSize, camera1ImagePoints[1]);
  if(!found1_2)
    {
    std::cerr << "Checkboard1_2 corners not found!" << std::endl;
    exit(-1);
    }


  std::vector<std::vector<Point2f> > camera2ImagePoints(2);
  bool found2_1 = findChessboardCorners(camera2image1, boardSize, camera2ImagePoints[0]);
  if(!found2_1)
    {
    std::cerr << "Checkboard2_1 corners not found!" << std::endl;
    exit(-1);
    }
  bool found2_2 = findChessboardCorners(camera2image2, boardSize, camera2ImagePoints[1]);
  if(!found2_2)
    {
    std::cerr << "Checkboard2_2 corners not found!" << std::endl;
    exit(-1);
    }

  namedWindow( "Corners", 1 );

  drawChessboardCorners(camera1image1, boardSize, camera1ImagePoints[0], found1_1);
  //imshow("Corners", image1);
  //waitKey(0);

  drawChessboardCorners(camera1image2, boardSize, camera1ImagePoints[1], found1_2);
  //imshow("Corners", image2);
  //waitKey(0);
  drawChessboardCorners(camera2image1, boardSize, camera2ImagePoints[0], found2_1);
  drawChessboardCorners(camera2image2, boardSize, camera2ImagePoints[1], found2_2);

  /*
  // Improve detection
  cornerSubPix(img, corners, Size(11,11), Size(-1,-1),
                        TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,
                                    30, 0.01));
  */
  std::vector<std::vector<Point3f> > objectPoints(2);
  objectPoints[0] = Create3DChessboardCorners(boardSize, squareSize);
  objectPoints[1] = Create3DChessboardCorners(boardSize, squareSize);

  Mat cameraMatrix[2];
  cameraMatrix[0] = Mat::eye(3, 3, CV_32FC1);
  cameraMatrix[1] = Mat::eye(3, 3, CV_32FC1);

  Mat distortionCoefficients[2];
  Mat rotationMatrix;
  Mat translationVector(3,1,CV_32FC1);
  Mat essentialMatrix;
  Mat fundamentalMatrix;


  double rms = stereoCalibrate(objectPoints, camera1ImagePoints, camera2ImagePoints,
                  cameraMatrix[0], distortionCoefficients[0],
                  cameraMatrix[1], distortionCoefficients[1],
                  imageSize, rotationMatrix, translationVector, essentialMatrix, fundamentalMatrix,
                  TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
                  CV_CALIB_FIX_ASPECT_RATIO +
                  CV_CALIB_ZERO_TANGENT_DIST +
                  CV_CALIB_SAME_FOCAL_LENGTH +
                  CV_CALIB_RATIONAL_MODEL +
                  CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);


  std::cout << "K1: " << cameraMatrix[0] << std::endl;
  std::cout << "K2: " << cameraMatrix[1] << std::endl;
  
  std::cout << "R: " << rotationMatrix << std::endl;
  std::cout << "T: " << translationVector << std::endl;
  // R is FROM camera1 TO camera2 ?

  /*
  // Draw image2 ontop of image1
  for(unsigned int x = 0; x < imageSize.width; x++)
    {
      for(unsigned int y = 0; y < imageSize.height; y++)
	{
	}
    }
  */
  
  // Trying to draw image1 ontop of image2
  // "Forward" projection - Loop over image 1 pixels
  Mat R1 = Mat::eye(3,3, CV_32FC1);
  Mat T1 = Mat::zeros(3,1, CV_32FC1);
  
  Mat P1 = ConstructP(R1, T1, cameraMatrix[0]);
  
  Mat P1inv(4,3,CV_32FC1);
  invert(P1,P1inv, DECOMP_SVD);
  
  //Mat P2 = ConstructP(GetFloat(rotationMatrix), GetFloat(translationVector), cameraMatrix[1]);
  Mat P2 = ConstructP(R1, GetFloat(translationVector), cameraMatrix[1]);
  Mat P2inv(4,3,CV_32FC1);
  invert(P2,P2inv, DECOMP_SVD);
  
  Mat outputImage(imageSize.height, imageSize.width, CV_8UC1);
  
  std::cout << "P1: "<< P1 << std::endl;
  std::cout << "P2: "<< P2 << std::endl;
  
  Mat point(4,1,CV_32FC1);
  point.at<float>(0,0) = 0;
  point.at<float>(1,0) = 0;
  point.at<float>(2,0) = 1;
  point.at<float>(3,0) = 1;
  std::cout << P2 * point << std::endl;
  
  for(unsigned int x = 0; x < imageSize.width; x++)
    {
      for(unsigned int y = 0; y < imageSize.height; y++)
	{
	Mat image1pixel(3,1,CV_32FC1);
	image1pixel.at<float>(0,0) = x;
	image1pixel.at<float>(1,0) = y;
	image1pixel.at<float>(2,0) = 1;
      
	Mat image2pixel(3,1,CV_32FC1);
	image2pixel = P2 * P1inv * image1pixel; // ziyan
	//image2pixel = P1 * P2inv * image1pixel;
      
	// divide image2pixel by last element
	image2pixel.at<float>(0,0) = round(image2pixel.at<float>(0,0)/image2pixel.at<float>(2,0));
	image2pixel.at<float>(1,0) = round(image2pixel.at<float>(1,0)/image2pixel.at<float>(2,0));
	
	//std::cout << image2pixel << std::endl;
	
	if(image2pixel.at<float>(0,0) < 0 || image2pixel.at<float>(0,0) >= imageSize.height ||
	  image2pixel.at<float>(1,0) < 0 || image2pixel.at<float>(1,0) >= imageSize.width)
	{
	  continue;
	}
	
	//std::cout << "image1pixel: " << image1pixel << std::endl;
	// the output image consists of pixels from image1
	outputImage.at<unsigned char>(image2pixel.at<float>(0,0), image2pixel.at<float>(1,0)) = 
		  camera1image1.at<unsigned char>(image1pixel.at<float>(0,0),image1pixel.at<float>(1,0));
	}
    }
  
  imwrite("output.jpg", outputImage);

  
  return 0;
}

Mat ConstructP(Mat R, Mat T, Mat Kin)
{
  Mat P(3,4,CV_32FC1);
  for(unsigned int i = 0; i < 3; i++)
    {
    for(unsigned int j = 0; j < 3; j++)
      {
      P.at<float>(i,j) = R.at<float>(i,j);
      }
    }
    
  //std::cout << P << std::endl;
    
  for(unsigned int row = 0; row < 3; row++)
    {
    P.at<float>(row,3) = T.at<float>(row,0);
    }
    
  //std::cout << P << std::endl;
  
  Mat K(3,3,CV_32FC1);
  
  for(unsigned int i = 0; i < 3; i++)
    {
    for(unsigned int j = 0; j < 3; j++)
      {
      K.at<float>(i,j) = Kin.at<double>(i,j);
      }
    }
    
  std::cout << P << std::endl;
    
  //std::cout << "Kin: " << Kin << std::endl;
  //std::cout << "K: " << K << std::endl;
    
  P = K * P;
  
  std::cout << P << std::endl;
  
  return P;
}

Mat GetFloat(Mat M)
{
  if(M.depth() == 5)
  {
    return M;
  }
  if(M.depth() == 6)
  {
    Mat K(M.rows,M.cols,CV_32FC1);
    for(unsigned int row = 0; row < 3; row++)
    {
      for(unsigned int col = 0; col < 3; col++)
	{
	K.at<float>(row,col) = M.at<double>(row,col);
	}
    }
    return K;
  }
  else
  {
    std::cerr << "Check the type of M! It is depth " << M.depth() << " !" << std::endl;
    exit(-1);
  }
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