#include <itkBackwardDifferenceOperator.h>

int main(int, char*[])
{
  typedef itk::BackwardDifferenceOperator<float, 2> BackwardDifferenceOperatorType;
  BackwardDifferenceOperatorType backwardDifferenceOperator;
  backwardDifferenceOperator.SetDirection(0); // Create the operator for the X axis derivative
  itk::Size<2> radius;
  radius.Fill(1);
  backwardDifferenceOperator.CreateToRadius(radius);

  std::cout << "Size: " << backwardDifferenceOperator.GetSize() << std::endl;

  std::cout << backwardDifferenceOperator << std::endl;

  for(unsigned int i = 0; i < 9; i++)
  {
    std::cout << backwardDifferenceOperator.GetOffset(i) << " " << backwardDifferenceOperator.GetElement(i) << std::endl;
  }
  return EXIT_SUCCESS;
}
