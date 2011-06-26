#include "cv.h"
#include "highgui.h"
#include <stdio.h>

#include <sstream>
#include <iostream>

int main()
{
  CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
  IplImage* frame = cvQueryFrame( capture );
  cvSaveImage("image.jpg",frame);
  cvReleaseCapture( &capture );
  cvDestroyWindow( "mywindow" );
  return 0;
}