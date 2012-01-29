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

int main(int argc, char** argv)
{
  Size boardSize(7,7);

  //std::string imageFileName = argv[1];
  std::string image1FileName = "board1.jpg";

  //std::string imageFileName = argv[2];
  std::string image2FileName = "board2.jpg";

  bool displayCorners = false;//true;
  const int maxScale = 2; // what is this?
  const float squareSize = 1.f;


  vector<vector<Point3f> > objectPoints;

  int numberOfImages = 2;

  vector<vector<Point2f> > imagePoints[2];
  imagePoints[0].resize(numberOfImages);
  imagePoints[1].resize(numberOfImages);

  Mat image1 = imread(image1FileName, 0);
  Mat image2 = imread(image2FileName, 0);

  Size imageSize = image1.size();

  bool found = false;
  vector<Point2f>& corners = imagePoints[k][j];
  for( int scale = 1; scale <= maxScale; scale++ )
  {
      Mat timg;
      if( scale == 1 )
      {
          timg = img;
      }
      else
      {
          resize(img, timg, Size(), scale, scale);
      }
      found = findChessboardCorners(timg, boardSize, corners,
          CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
      if( found )
      {
          if( scale > 1 )
          {
              Mat cornersMat(corners);
              cornersMat *= 1./scale;
          }
          break;
      }
  }

  if( displayCorners )
  {
      std::cout << filename << std::endl;
      Mat cimg, cimg1;
      cvtColor(img, cimg, CV_GRAY2BGR);
      drawChessboardCorners(cimg, boardSize, corners, found);
      double sf = 640./MAX(img.rows, img.cols);
      resize(cimg, cimg1, Size(), sf, sf);
      imshow("corners", cimg1);
      char c = (char)waitKey(500);
      if( c == 27 || c == 'q' || c == 'Q' ) //Allow ESC to quit
        {
          exit(-1);
        }
  }
  else
  {
      putchar('.');
  }

  if( !found )
  {
    std::cerr << "Checkboards not found!" << std::endl;
    exit(-1);
  }

  cornerSubPix(img, corners, Size(11,11), Size(-1,-1),
                        TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,
                                    30, 0.01));

  imagePoints[0].resize(nimages);
  imagePoints[1].resize(nimages);
  objectPoints.resize(nimages);

  for( i = 0; i < nimages; i++ )
  {
      for( j = 0; j < boardSize.height; j++ )
      {
          for( k = 0; k < boardSize.width; k++ )
          {
              objectPoints[i].push_back(Point3f(j*squareSize, k*squareSize, 0));
          }
      }
  }

  Mat cameraMatrix[2];
  cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
  cameraMatrix[1] = Mat::eye(3, 3, CV_64F);

  Mat distortionCoefficients[2];
  Mat rotationMatrix;
  Mat translationVector;
  Mat essentialMatrix;
  Mat fundamentalMatrix;

  double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                  cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  imageSize, R, T, E, F,
                  TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
                  CV_CALIB_FIX_ASPECT_RATIO +
                  CV_CALIB_ZERO_TANGENT_DIST +
                  CV_CALIB_SAME_FOCAL_LENGTH +
                  CV_CALIB_RATIONAL_MODEL +
                  CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);
  std::cout << "done with RMS error=" << rms << std::endl;


// CALIBRATION QUALITY CHECK
// because the output fundamental matrix implicitly
// includes all the output information,
// we can check the quality of calibration using the
// epipolar geometry constraint: m2^t*F*m1=0
    double err = 0;
    int npoints = 0;
    vector<Vec3f> lines[2];
    for( i = 0; i < nimages; i++ )
    {
        int npt = (int)imagePoints[0][i].size();
        Mat imgpt[2];
        for( k = 0; k < 2; k++ )
        {
            imgpt[k] = Mat(imagePoints[k][i]);
            undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
            computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
        }
        for( j = 0; j < npt; j++ )
        {
            double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
                                imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
                           fabs(imagePoints[1][i][j].x*lines[0][j][0] +
                                imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }
    cout << "average reprojection err = " <<  err/npoints << endl;

    // save intrinsic parameters
    FileStorage fs("intrinsics.yml", CV_STORAGE_WRITE);
    if( fs.isOpened() )
    {
        fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
            "M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
        fs.release();
    }
    else
        cout << "Error: can not save the intrinsic parameters\n";

    Mat R1, R2, P1, P2, Q;
    Rect validRoi[2];

    stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  imageSize, R, T, R1, R2, P1, P2, Q,
                  1, imageSize, &validRoi[0], &validRoi[1]);

    fs.open("extrinsics.yml", CV_STORAGE_WRITE);
    if( fs.isOpened() )
    {
        fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
        fs.release();
    }
    else
        cout << "Error: can not save the intrinsic parameters\n";

    // OpenCV can handle left-right
    // or up-down camera arrangements
    bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

// COMPUTE AND DISPLAY RECTIFICATION
    if( !showRectified )
        return;

    Mat rmap[2][2];
// IF BY CALIBRATED (BOUGUET'S METHOD)
    if( useCalibrated )
    {
        // we already computed everything
    }
// OR ELSE HARTLEY'S METHOD
    else
 // use intrinsic parameters of each camera, but
 // compute the rectification transformation directly
 // from the fundamental matrix
    {
        vector<Point2f> allimgpt[2];
        for( k = 0; k < 2; k++ )
        {
            for( i = 0; i < nimages; i++ )
                std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
        }
        F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
        Mat H1, H2;
        stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);

        R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
        R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
        P1 = cameraMatrix[0];
        P2 = cameraMatrix[1];
    }

    //Precompute maps for cv::remap()
    initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

    Mat canvas;
    double sf;
    int w, h;
    if( !isVerticalStereo )
    {
        sf = 600./MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width*sf);
        h = cvRound(imageSize.height*sf);
        canvas.create(h, w*2, CV_8UC3);
    }
    else
    {
        sf = 300./MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width*sf);
        h = cvRound(imageSize.height*sf);
        canvas.create(h*2, w, CV_8UC3);
    }

    for( i = 0; i < nimages; i++ )
    {
        for( k = 0; k < 2; k++ )
        {
            Mat img = imread(goodImageList[i*2+k], 0), rimg, cimg;
            remap(img, rimg, rmap[k][0], rmap[k][1], CV_INTER_LINEAR);
            cvtColor(rimg, cimg, CV_GRAY2BGR);
            Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*k, 0, w, h)) : canvas(Rect(0, h*k, w, h));
            resize(cimg, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
            if( useCalibrated )
            {
                Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
                          cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
                rectangle(canvasPart, vroi, Scalar(0,0,255), 3, 8);
            }
        }

        if( !isVerticalStereo )
            for( j = 0; j < canvas.rows; j += 16 )
                line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
        else
            for( j = 0; j < canvas.cols; j += 16 )
                line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
        imshow("rectified", canvas);
        char c = (char)waitKey();
        if( c == 27 || c == 'q' || c == 'Q' )
            break;
    }

  return 0;
}

