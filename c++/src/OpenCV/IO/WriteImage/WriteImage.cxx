#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <string>

int main( int argc, char* argv[])
{
  std::string inputFileName = argv[1];
  std::string outputFileName = argv[2];

  cv::Mat image = cv::imread(inputFileName, 1);

  cv::imwrite(outputFileName, image);

  return 0;
}
