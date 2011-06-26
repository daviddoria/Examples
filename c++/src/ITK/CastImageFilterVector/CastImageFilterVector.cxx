// This is not correct. Please see CastImageFilter, CastImageFilter/CastImageFilter_Vector, or VectorCastImageFilter
#include "itkImage.h"
#include "itkCovariantVector.h"
#include "itkImageFileReader.h"
#include "itkCastImageFilter.h"

typedef itk::CovariantVector<float,3> FloatVectorType;
typedef itk::Image<FloatVectorType, 2> FloatVectorImageType;

typedef itk::CovariantVector<unsigned char,3> UnsignedCharVectorType;
typedef itk::Image<UnsignedCharVectorType, 2> UnsignedCharVectorImageType;

template<typename T>
void CastImage(typename T::Pointer image)
{
  typedef itk::CastImageFilter< T, UnsignedCharVectorImageType > CastFilterType;
  typename CastFilterType::Pointer castFilter = CastFilterType::New();
  castFilter->SetInput(image);
  castFilter->Update();
/*
 "The type you declared image as, "typename T:ointer" is not a deducible context for T, because given an arbitrary type, the compiler has no way of figuring out the "correct" T such that T:ointer is the given type. hence, T cannot be deduced."
 */
}

int main(int argc, char *argv[])
{
  FloatVectorImageType::Pointer floatVectorImage = FloatVectorImageType::New();
  //CastImage<FloatVectorImageType>(floatVectorImage);
  CastImage(floatVectorImage);

  return EXIT_SUCCESS;
}