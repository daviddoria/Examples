#include "cv.h"
#include "highgui.h"
#include <stdio.h>

#include <sstream>
#include <iostream>

int main()
{
  CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
  if( !capture )
  {
    fprintf( stderr, "ERROR: capture is NULL \n" );
    getchar();
    return -1;
  }

  // Create a window in which the captured images will be presented
  cvNamedWindow( "mywindow", CV_WINDOW_AUTOSIZE );

  // Get frames
  while((cvWaitKey(10) & 255) != 27) // press 'escape' to end
  {
    IplImage* frame = cvQueryFrame( capture );

    cvShowImage( "mywindow", frame );
  }

  cvReleaseCapture( &capture );
  cvDestroyWindow( "mywindow" );
  return 0;
}       