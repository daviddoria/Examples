// OpenCV
#include "cv.h"
#include "opencv2/core/eigen.hpp"

// STL
#include <iostream>

// Eigen
#include <Eigen/Dense>

void OpenCVtoEigen();
void EigenToOpenCV();

int main()
{
  OpenCVtoEigen();
  EigenToOpenCV();

  return 0;
}

void OpenCVtoEigen()
{
  cv::Mat M(3,3,cv::DataType<float>::type);

  for(unsigned int i = 0; i < 3; i++)
    {
    for(unsigned int j = 0; j < 3; j++)
      {
      M.at<float>(i,j) = 2;
      }
    }

  for(unsigned int i = 0; i < 3; i++)
    {
    for(unsigned int j = 0; j < 3; j++)
      {
      std::cout << M.at<float>(i,j) << std::endl;
      }
    }

  Eigen::MatrixXd m(3,3);
  
  cv::cv2eigen(M, m);
}

void EigenToOpenCV()
{

  Eigen::MatrixXd m(2,2);
  m(0,0) = 3;
  m(1,0) = 2.5;
  m(0,1) = -1;
  m(1,1) = m(1,0) + m(0,1);
  std::cout << m << std::endl;
}
