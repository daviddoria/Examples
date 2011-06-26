#include <itkImage.h>

int main(int, char*[])
{
  typedef itk::Image<unsigned char, 2> ImageType;
  ImageType::Pointer image = ImageType::New();

  std::cout << "image is type: " << image->GetNameOfClass() << std::endl;

  return EXIT_SUCCESS;
}
