#include "itkVectorImage.h"
#include "itkCastImageFilter.h"

int main(int argc, char *argv[])
{
  typedef itk::VectorImage<unsigned char, 2>  UnsignedCharVectorImageType;
  typedef itk::VectorImage<float, 2>  FloatVectorImageType;

  FloatVectorImageType::Pointer image = FloatVectorImageType::New();

  typedef itk::CastImageFilter< FloatVectorImageType, UnsignedCharVectorImageType > CastImageFilterType;
  CastImageFilterType::Pointer castImageFilter = CastImageFilterType::New();
  castImageFilter->SetInput(image);
  castImageFilter->Update();

  return EXIT_SUCCESS;
}
