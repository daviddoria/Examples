#include <iostream>

#include <Eigen/Dense>

int main(int argc, char *argv[])
{

  Eigen::VectorXd v(2);
  v(0) = 3;
  v(1) = 2.5;

  std::cout << v << std::endl;
  return 0;
}
