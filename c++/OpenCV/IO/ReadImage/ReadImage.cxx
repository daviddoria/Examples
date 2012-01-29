#include <opencv2/highgui/highgui.hpp>
#include "cv.h"

#include <iostream>


int main(int, char*[])
{
  cv::Mat image = cv::imread("calleigh.jpg", CV_LOAD_IMAGE_COLOR); // or CV_LOAD_IMAGE_GRAYSCALE

  if(image.empty())
  {
    std::cout << "Can't the image" << std::endl;
    return -1;
  }

  return 0;
}
