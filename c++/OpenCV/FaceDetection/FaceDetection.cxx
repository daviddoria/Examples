#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

void detectAndDrawNested( Mat& img,
                   CascadeClassifier& cascade, CascadeClassifier& nestedCascade);

String frontalFaceCascadeName = "/home/doriad/src/OpenCV/data/haarcascades/haarcascade_frontalface_alt.xml";
String eyeCascadeName = "/home/doriad/src/OpenCV/data/haarcascades/haarcascade_eye.xml";
String mouthCascadeName = "/home/doriad/src/OpenCV/data/haarcascades/haarcascade_mcs_mouth.xml";
String noseCascadeName = "/home/doriad/src/OpenCV/data/haarcascades/haarcascade_mcs_nose.xml";
String leftEyeCascadeName = "/home/doriad/src/OpenCV/data/haarcascades/haarcascade_mcs_lefteye.xml";
String rightEyeCascadeName = "/home/doriad/src/OpenCV/data/haarcascades/haarcascade_mcs_righteye.xml";

int main( int argc, const char** argv )
{
  CvCapture* capture = 0;
  Mat frame, frameCopy, image;

  CascadeClassifier frontalFaceCascade;
  CascadeClassifier eyeCascade;
  CascadeClassifier mouthCascade;
  CascadeClassifier noseCascade;
  CascadeClassifier leftEyeCascade;
  CascadeClassifier rightEyeCascade;

  frontalFaceCascade.load( frontalFaceCascadeName );
  eyeCascade.load( eyeCascadeName );
  leftEyeCascade.load( leftEyeCascadeName );
  rightEyeCascade.load( rightEyeCascadeName );
  mouthCascade.load( mouthCascadeName );
  noseCascade.load( noseCascadeName );

  image = imread(argv[1], 1 );

  if(image.empty())
  {
    std::cout << "Couldn't read image." << std::endl;
    return -1;
  }

  cvNamedWindow( "result", 1 );

  //detectAndDrawNested( image, frontalFaceCascade, eyeCascade);
  //detectAndDrawNested( image, frontalFaceCascade, mouthCascade);
  detectAndDrawNested( image, frontalFaceCascade, noseCascade);
  //detectAndDrawNested( image, frontalFaceCascade, rightEyeCascade);
  //detectAndDrawNested( image, frontalFaceCascade, leftEyeCascade);

  waitKey(0);

  cvDestroyWindow("result");

  return 0;
}

void detectAndDrawNested( Mat& img,
                   CascadeClassifier& cascade, CascadeClassifier& nestedCascade)
{
  double scale = 1.0;

  Scalar blue = CV_RGB(0,0,255);
  Scalar red = CV_RGB(255,0,0);
  Mat gray, smallImg( cvRound (img.rows/scale), cvRound(img.cols/scale), CV_8UC1 );

  cvtColor( img, gray, CV_BGR2GRAY );
  resize( gray, smallImg, smallImg.size(), 0, 0, INTER_LINEAR );
  equalizeHist( smallImg, smallImg );

  vector<Rect> faces;
  cascade.detectMultiScale( smallImg, faces,
      1.1, 2, 0
      //|CV_HAAR_FIND_BIGGEST_OBJECT
      //|CV_HAAR_DO_ROUGH_SEARCH
      |CV_HAAR_SCALE_IMAGE
      ,
      Size(30, 30) );

  int i = 0;
  for( vector<Rect>::const_iterator r = faces.begin(); r != faces.end(); r++, i++ )
  {
    Point center;
    center.x = cvRound((r->x + r->width*0.5)*scale);
    center.y = cvRound((r->y + r->height*0.5)*scale);

    int radius;
    radius = cvRound((r->width + r->height)*0.25*scale);
    circle( img, center, radius, red, 3, 8, 0 );

    if( nestedCascade.empty() )
    {
      continue;
    }
    Mat smallImgROI = smallImg(*r);
    vector<Rect> nestedObjects;
    nestedCascade.detectMultiScale( smallImgROI, nestedObjects,
        1.1, 2, 0
        //|CV_HAAR_FIND_BIGGEST_OBJECT
        //|CV_HAAR_DO_ROUGH_SEARCH
        //|CV_HAAR_DO_CANNY_PRUNING
        |CV_HAAR_SCALE_IMAGE
        ,
        Size(30, 30), r->tl() );
    for( vector<Rect>::const_iterator nr = nestedObjects.begin(); nr != nestedObjects.end(); nr++ )
    {
      center.x = cvRound((r->x + nr->x + nr->width*0.5)*scale);
      center.y = cvRound((r->y + nr->y + nr->height*0.5)*scale);
      radius = cvRound((nr->width + nr->height)*0.25*scale);
      circle( img, center, radius, blue, 3, 8, 0 );
    }
  }
  cv::imshow( "result", img );
}
