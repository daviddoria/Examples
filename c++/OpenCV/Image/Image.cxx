#include "cv.h"

#include <iostream>

int main()
{
  // Create an image of scalars (the '1' as the last parameter)
  IplImage* image = cvCreateImage(cvSize(20,10), IPL_DEPTH_8U, 1);

  // Access a pixel
  CvScalar pixel = cvGet2D(image, 4, 5);

  // Access a component of the pixel
  std::cout << pixel.val[0] << std::endl;

  return 0;
}

