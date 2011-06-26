#include "itkImage.h"
#include "itkVector.h"
#include "itkVectorCastImageFilter.h"

int main(int argc, char *argv[])
{
  typedef itk::Image<itk::Vector<unsigned char, 3>, 2>  UnsignedCharVectorImageType;
  typedef itk::Image<itk::Vector<float, 3>, 2>  FloatVectorImageType;

  FloatVectorImageType::Pointer image = FloatVectorImageType::New();

  typedef itk::VectorCastImageFilter< FloatVectorImageType, UnsignedCharVectorImageType > VectorCastImageFilterType;
  VectorCastImageFilterType::Pointer vectorCastImageFilter = VectorCastImageFilterType::New();
  vectorCastImageFilter->SetInput(image);
  vectorCastImageFilter->Update();

  return EXIT_SUCCESS;
}
