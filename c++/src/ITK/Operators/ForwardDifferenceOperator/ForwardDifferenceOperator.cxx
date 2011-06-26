#include <itkForwardDifferenceOperator.h>

int main(int, char*[])
{
  typedef itk::ForwardDifferenceOperator<float, 2> ForwardDifferenceOperatorType;
  ForwardDifferenceOperatorType forwardDifferenceOperator;
  forwardDifferenceOperator.SetDirection(0); // Create the operator for the X axis derivative
  itk::Size<2> radius;
  radius.Fill(1);
  forwardDifferenceOperator.CreateToRadius(radius);

  std::cout << "Size: " << forwardDifferenceOperator.GetSize() << std::endl;

  std::cout << forwardDifferenceOperator << std::endl;

  for(unsigned int i = 0; i < 9; i++)
  {
    std::cout << forwardDifferenceOperator.GetOffset(i) << " " << forwardDifferenceOperator.GetElement(i) << std::endl;
  }
  return EXIT_SUCCESS;
}
