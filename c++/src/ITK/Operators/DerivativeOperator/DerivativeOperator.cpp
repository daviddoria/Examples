#include <itkDerivativeOperator.h>

int main(int, char*[])
{
  typedef itk::DerivativeOperator<float, 2> DerivativeOperatorType;
  DerivativeOperatorType derivativeOperator;
  derivativeOperator.SetDirection(0); // Create the operator for the X axis derivative
  itk::Size<2> radius;
  radius.Fill(1);
  derivativeOperator.CreateToRadius(radius);

  std::cout << "Size: " << derivativeOperator.GetSize() << std::endl;

  std::cout << derivativeOperator << std::endl;

  for(unsigned int i = 0; i < 9; i++)
  {
    std::cout << derivativeOperator.GetOffset(i) << " " << derivativeOperator.GetElement(i) << std::endl;
  }
  return EXIT_SUCCESS;
}
