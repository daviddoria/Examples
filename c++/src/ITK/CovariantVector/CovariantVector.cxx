#include <itkCovariantVector.h>

int main(int, char*[])
{
  typedef itk::CovariantVector<double, 3> VectorType;
  VectorType v;
  v[0] = 1.0;
  v[1] = 2.0;
  v[2] = 3.0;
  
  std::cout << "v: " << v << std::endl;

  v.Normalize();
  std::cout << "v: " << v << std::endl;
  
  return EXIT_SUCCESS;
}
