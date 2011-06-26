#include "itkImage.h"
#include "itkCovariantVector.h"
#include "itkImageFileReader.h"
#include "itkCastImageFilter.h"

typedef itk::CovariantVector<float,3> FloatVectorType;
typedef itk::Image<FloatVectorType, 2> FloatVectorImageType;

typedef itk::CovariantVector<unsigned char,3> UnsignedCharVectorType;
typedef itk::Image<UnsignedCharVectorType, 2> UnsignedCharVectorImageType;

int main(int argc, char *argv[])
{
  FloatVectorImageType::Pointer floatVectorImage = FloatVectorImageType::New();

  typedef itk::CastImageFilter< FloatVectorImageType, UnsignedCharVectorImageType > CastFilterType;
  CastFilterType::Pointer castFilter = CastFilterType::New();
  castFilter->SetInput(floatVectorImage);
  castFilter->Update();

  return EXIT_SUCCESS;
}