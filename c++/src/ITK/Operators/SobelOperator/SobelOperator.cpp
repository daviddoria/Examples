#include <itkSobelOperator.h>

int main(int, char*[])
{
  typedef itk::SobelOperator<float, 2> SobelOperatorType;
  SobelOperatorType sobelOperator;
  sobelOperator.SetDirection(0); // Create the operator for the X axis derivative
  itk::Size<2> radius;
  radius.Fill(1);
  sobelOperator.CreateToRadius(radius);

  std::cout << "Size: " << sobelOperator.GetSize() << std::endl;

  std::cout << sobelOperator << std::endl;

  for(unsigned int i = 0; i < 9; i++)
  {
    std::cout << sobelOperator.GetOffset(i) << " " << sobelOperator.GetElement(i) << std::endl;
  }
  return EXIT_SUCCESS;
}
