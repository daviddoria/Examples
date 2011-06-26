#include <itkImage.h>
#include <itkWindowedSincInterpolateImageFunction.h>

int main(int, char*[])
{
  typedef itk::Function::HammingWindowFunction<5> HammingWindowFunctionType;
  HammingWindowFunctionType hammingWindowFunction;

  std::cout << "Value at 1.3: " << hammingWindowFunction(1.3) << std::endl;

  return EXIT_SUCCESS;
}
