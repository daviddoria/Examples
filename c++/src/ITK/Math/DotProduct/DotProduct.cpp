#include <itkVector.h>

int main(int, char*[])
{
  typedef itk::Vector<double, 3> VectorType;
  
  VectorType v1;
  v1[0] = 1.0;
  v1[1] = 2.0;
  v1[2] = 3.0;
  
  VectorType v2;
  v2[0] = 1.0;
  v2[1] = 2.0;
  v2[2] = 3.0;
  
  VectorType dotproduct = v1*v2;
  std::cout << "dot(v1,v2): " << dotproduct << std::endl;

  return EXIT_SUCCESS;
}
