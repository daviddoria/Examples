#include <itkGaussianOperator.h>

int main(int, char*[])
{
  typedef itk::GaussianOperator<float, 2> GaussianOperatorType;
  GaussianOperatorType gaussianOperator;
  gaussianOperator.SetDirection(0); // Create the operator for the X axis derivative
  itk::Size<2> radius;
  radius.Fill(1);
  gaussianOperator.CreateToRadius(radius);

  std::cout << "Size: " << gaussianOperator.GetSize() << std::endl;

  std::cout << gaussianOperator << std::endl;

  for(unsigned int i = 0; i < 9; i++)
  {
    std::cout << gaussianOperator.GetOffset(i) << " " << gaussianOperator.GetElement(i) << std::endl;
  }
  return EXIT_SUCCESS;
}
