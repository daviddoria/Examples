#include "opencv2/calib3d/calib3d.hpp"

#include <iostream>

int main()
{
  cv::Scalar myScalar;

  myScalar.val[0] = 1;
  myScalar.val[1] = 2;
  myScalar.val[2] = 3;

  std::cout << myScalar.val[0] << " " << myScalar.val[1] << " " << myScalar.val[2] << std::endl;

  return 0;
}