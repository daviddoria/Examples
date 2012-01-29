#include <iostream>

#include <Eigen/Geometry>
#include <Eigen/Dense> // for Vector

void CreateHomeogeneous();
void ConvertToHomeogeneous();
void ConvertFromHomeogeneous();

int main(int argc, char *argv[])
{
  //CreateHomeogeneous();
  ConvertToHomeogeneous();
  ConvertFromHomeogeneous();

  return 0;
}

void CreateHomeogeneous()
{
  /*
  Eigen::Homogeneous<Eigen::Vector3d, Eigen::Vertical> h;
  h(0,0) = 3;
  h(1,0) = 2.5;

  std::cout << h << std::endl;
  */
}

void ConvertToHomeogeneous()
{
  Eigen::Vector3d v;
  v(0) = 0;
  v(1) = 1;
  v(2) = 2;

  std::cout << v << std::endl;

  //std::cout << v.homogeneous() << std::endl;
  Eigen::Vector4d h = v.homogeneous();
  std::cout << h << std::endl;

  Eigen::Vector3d::HomogeneousReturnType h2 = v.homogeneous();
  std::cout << h2 << std::endl;
}

void ConvertFromHomeogeneous()
{
  Eigen::Vector3d v;
  v(0) = 0;
  v(1) = 1;
  v(2) = 2;

  std::cout << v << std::endl;

  //std::cout << v.homogeneous() << std::endl;
  Eigen::Vector4d h = v.homogeneous();
  std::cout << h << std::endl;
  h(3) = .5;
  std::cout << h << std::endl;

  // Works, but h2 seems to be read only
  //Eigen::Vector3d::HomogeneousReturnType h2 = v.homogeneous();
  //std::cout << h2 << std::endl;

  // Modified - this doesn't work
  //h2(3) = .5;
  //std::cout << h2 << std::endl;
}
