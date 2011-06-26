#include <itkGaussianDerivativeOperator.h>

int main(int, char*[])
{
  typedef itk::GaussianDerivativeOperator<float, 2> GaussianDerivativeOperatorType;
  GaussianDerivativeOperatorType gaussianDerivativeOperator;
  gaussianDerivativeOperator.SetDirection(0); // Create the operator for the X axis derivative
  itk::Size<2> radius;
  radius.Fill(1);
  gaussianDerivativeOperator.CreateToRadius(radius);

  std::cout << "Size: " << gaussianDerivativeOperator.GetSize() << std::endl;

  std::cout << gaussianDerivativeOperator << std::endl;

  for(unsigned int i = 0; i < 9; i++)
  {
    std::cout << gaussianDerivativeOperator.GetOffset(i) << " " << gaussianDerivativeOperator.GetElement(i) << std::endl;
  }
  return EXIT_SUCCESS;
}
