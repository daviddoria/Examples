#include <itkImage.h>
#include <itkRGBPixel.h>

int main(int argc, char *argv[])
{
  typedef itk::RGBPixel<unsigned char> RGBPixelType;
  typedef itk::Image<RGBPixelType> RGBImageType;
  RGBImageType::Pointer image = RGBImageType::New();
  
  return EXIT_SUCCESS;
}
