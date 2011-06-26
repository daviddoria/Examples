#include <itkLaplacianOperator.h>

int main(int, char*[])
{
  typedef itk::LaplacianOperator<float, 2> LaplacianOperatorType;
  LaplacianOperatorType laplacianOperator;
  itk::Size<2> radius;
  radius.Fill(1);
  laplacianOperator.CreateToRadius(radius);

  std::cout << "Size: " << laplacianOperator.GetSize() << std::endl;

  std::cout << laplacianOperator << std::endl;

  for(unsigned int i = 0; i < laplacianOperator.GetSize()[0] * laplacianOperator.GetSize()[1]; i++)
  {
    std::cout << laplacianOperator.GetOffset(i) << " " << laplacianOperator.GetElement(i) << std::endl;
  }
  return EXIT_SUCCESS;
}
