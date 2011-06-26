#include "itkImage.h"

int main(int, char *[])
{
  typedef itk::Image<unsigned char, 2> ImageType;
  ImageType::Pointer image = ImageType::New();

  ImageType::PixelType pixel = 0;
  
  std::cout << pixel << std::endl;

  return EXIT_SUCCESS;
}
