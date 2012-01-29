#include <iostream>

#include <Eigen/Dense>

void VariableLength();
void FixedLength();
void Operations();
void Norm();

int main(int argc, char *argv[])
{

  //VariableLength();
  //FixedLength();
  //Operations();
  Norm();
  
  return 0;
}

void VariableLength()
{
  Eigen::VectorXd v(2);
  v(0) = 3;
  v(1) = 2.5;

  std::cout << v << std::endl;

}

void FixedLength()
{
  Eigen::Vector3d v3(1,2,3);
  std::cout << v3 << std::endl;

  Eigen::Vector2d v2(1,2);
  std::cout << v2 << std::endl;

  v2(0) = 5;
  std::cout << v2 << std::endl;
}

void Operations()
{
  Eigen::Vector3d v1(1,2,3);
  std::cout << v1 << std::endl;

  Eigen::Vector3d v2(2,4,6);
  std::cout << v2 << std::endl;

  std::cout << v1+v2 << std::endl;

  std::cout << 5.0*v1 << std::endl;

}

void Norm()
{
  Eigen::Vector3d v1(1,2,3);
  std::cout << v1.norm() << std::endl;
}
