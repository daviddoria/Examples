#include "itkImageAdaptor.h"
#include "itkCovariantVector.h"
#include "itkCompose3DCovariantVectorImageFilter.h"

int main(int, char *[])
{
  // This doesn't work, but why do we need it anyway?

  itk::CovariantVector<float, 3> a = itk::Function::Compose3DCovariantVector<float>(1.0f,2.0f,3.0f);
  std::cout << a << std::endl;
  return EXIT_SUCCESS;
}
