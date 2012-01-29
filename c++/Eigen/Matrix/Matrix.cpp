#include <iostream>

#include <Eigen/Dense>

void Basic();
void ExtractColumn();

int main(int argc, char *argv[])
{

  ExtractColumn();
  return 0;
}


void Basic()
{
  Eigen::MatrixXd m(2,2);
  m(0,0) = 3;
  m(1,0) = 2.5;
  m(0,1) = -1;
  m(1,1) = m(1,0) + m(0,1);
  std::cout << m << std::endl;
  
}

void ExtractColumn()
{
  Eigen::MatrixXd m(2,2);
  m(0,0) = 3;
  m(1,0) = 2.5;
  m(0,1) = -1;
  m(1,1) = 7;

  Eigen::MatrixXd column0 = m.col(0);
  
}
