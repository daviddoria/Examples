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
  std::cout << "ConvertToHomeogeneous()" << std::endl;

  // Create a 3D vector
  Eigen::Vector3d v;
  v(0) = 0;
  v(1) = 1;
  v(2) = 2;

  std::cout << "Original 3D vector: " << v << std::endl;

  // Get the homogeneous version of the vector
  Eigen::Vector4d h = v.homogeneous();
  std::cout << "Homogeneous version (4D): " << h << std::endl;

}

void ConvertFromHomeogeneous()
{
  std::cout << "ConvertFromHomeogeneous()" << std::endl;

  // Create a 4D vector
  Eigen::Vector4d v;
  v(0) = 0;
  v(1) = 1;
  v(2) = 2;
  v(3) = .5;

  std::cout << "Original 4D homogeneous vector: " <<  v << std::endl;

  // Divide the vector by the last entry
  Eigen::Vector3d unhomog = v.hnormalized();
  std::cout << "Un-homogeneous vector: " << unhomog << std::endl;
}
