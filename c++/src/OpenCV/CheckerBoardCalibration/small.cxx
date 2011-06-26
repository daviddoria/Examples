#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <string>

using namespace cv;

static void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners);

int main( int argc, char** argv )
{
    Size boardSize(8,8);
    Size imageSize;
    float squareSize = 1.f;
    float aspectRatio = 1.f;
    Mat cameraMatrix;
    Mat distCoeffs;

    int nframes = 1;
    bool writeExtrinsics = false;
    bool writePoints = false;
    bool undistortImage = false;

    bool showUndistorted = false;


    std::string imageFileName = "board.jpg";

    namedWindow( "Image View", 1 );

    Mat view = imread(imageFileName, 1);

    imageSize = view.size();

    // Find the chessboard corners
    vector<vector<Point2f> > imagePoints(1);
    findChessboardCorners(view, boardSize, imagePoints[0]);

    std::vector<std::vector<Point3f> > objectPoints(1);
    objectPoints.resize(imagePoints.size(),objectPoints[0]);

    std::vector<Mat> rotationVectors;
    std::vector<Mat> translationVectors;
    distCoeffs = Mat::zeros(8, 1, CV_64F);
    cameraMatrix = Mat::eye(3, 3, CV_64F);

    int flags = 0;
    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                    distCoeffs, rotationVectors, translationVectors, flags|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);

    /*
    std::cout << "RMS error reported by calibrateCamera: " << rms << std::endl;

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    vector<Point2f> pointbuf;
    Mat viewGray;
    cvtColor(view, viewGray, CV_BGR2GRAY);

    bool found = findChessboardCorners( view, boardSize, pointbuf,
                CV_CALIB_CB_ADAPTIVE_THRESH & CV_CALIB_CB_FAST_CHECK & CV_CALIB_CB_NORMALIZE_IMAGE);


    // improve the found corners' coordinate accuracy
    cornerSubPix( viewGray, pointbuf, Size(11,11),
        Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

    drawChessboardCorners( view, boardSize, Mat(pointbuf), found );

    imshow("View", view);

    waitKey(0);

    cvDestroyWindow("View");
*/
    return 0;
}

static void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners)
{
    corners.resize(0);

    for( int i = 0; i < boardSize.height; i++ )
    {
      for( int j = 0; j < boardSize.width; j++ )
      {
        corners.push_back(Point3f(float(j*squareSize),
                                  float(i*squareSize), 0));
      }
    }
}
