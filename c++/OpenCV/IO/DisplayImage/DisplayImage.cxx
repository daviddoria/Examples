#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>

int main(int argc, char*argv[])
{
  // Read the image
  cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);

  if(image.empty())
  {
    std::cerr << "Can't read the image!" << std::endl;
    return -1;
  }

  // Display the image
  cv::namedWindow("image", 1);
  cv::imshow("matches", image);
  cv::waitKey(0);

  return 0;
}
