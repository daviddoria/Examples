#include "itkImage.h"
#include "itkNumericTraits.h"

int main(int, char *[])
{
  typedef itk::Image<unsigned char, 2> ImageType;
  ImageType::Pointer image = ImageType::New();

  ImageType::PixelType pixel = 0;

  std::cout << (int)pixel << std::endl;

  std::cout << (int)itk::NumericTraits< ImageType::PixelType >::Zero << std::endl;

  return EXIT_SUCCESS;
}
