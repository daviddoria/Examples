#include <iostream>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

int main(int argc, char *argv[])
{
  //Eigen::MatrixXd R(3,3); // Note you cannot use a dynamic sized matrix - there will be a compiler error.
  Eigen::Matrix3d R;
  R(0,0) = 0.209339;
  R(0,1) = 0.976875;
  R(0,2) = -0.043499;
  R(1,0) = -0.090657;
  R(1,1) = 0.063682;
  R(1,2) = 0.993844;
  R(2,0) = -0.973632;
  R(2,1) = 0.204106;
  R(2,2) = -0.101891;

  R *= -1.0f;
  std::cout << Eigen::AngleAxisd(R).axis() << std::endl;
  
  return 0;
}
