#include <itkVector.h>

int main(int, char*[])
{
  typedef itk::Vector<double, 3> VectorType;
  VectorType v;
  v[0] = 1.0;
  v[1] = 2.0;
  v[2] = 3.0;
  std::cout << "v: " << v << std::endl;

  VectorType v2;
  v2[0] = 2.0;
  v2[1] = 3.0;
  v2[2] = 4.0;

  std::cout << "v2 - v: " << v2 - v << std::endl;

  return EXIT_SUCCESS;
}
