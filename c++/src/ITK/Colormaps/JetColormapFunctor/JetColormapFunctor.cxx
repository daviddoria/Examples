#include "itkJetColormapFunctor.h"
#include "itkRGBPixel.h"

int main( int argc, char *argv[])
{
  typedef itk::RGBPixel<unsigned char> PixelType;
  typedef itk::Functor::JetColormapFunctor<float, PixelType> ColorMapType;
  ColorMapType::Pointer colormap = ColorMapType::New();

  colormap->SetMinimumInputValue(0.0);
  colormap->SetMaximumInputValue(1.0);
  std::cout << "0: " << colormap->operator()(0.0f) << std::endl;
  std::cout << "0.5: " << colormap->operator()(0.5f) << std::endl;
  std::cout << "1: " << colormap->operator()(1.0f) << std::endl;
  return EXIT_SUCCESS;
}
