#include "itkVectorImage.h"
#include "itkVector.h"
#include "itkVariableLengthVector.h"
#include "itkRigid2DTransform.h"
#include "itkUnaryFunctorImageFilter.h"

template< class TInput, class TOutput>
class RotateVectors
{
public:
  RotateVectors() {};
  ~RotateVectors() {};
  bool operator!=( const RotateVectors & ) const
    {
    return false;
    }
  bool operator==( const RotateVectors & other ) const
    {
    return !(*this != other);
    }
  inline TOutput operator()( const TInput & A ) const
    {
      typedef itk::Vector<double, 2> VectorType;
      VectorType v;
      v[0] = A[0];
      v[1] = A[1];

      typedef itk::Rigid2DTransform< float > TransformType;

      TransformType::Pointer transform = TransformType::New();
      transform->SetAngle(M_PI/2.0);

      VectorType outputV = transform->TransformVector(v);
      TOutput transformedVector;
      transformedVector.SetSize(2);
      transformedVector[0] = outputV[0];
      transformedVector[1] = outputV[1];

      return transformedVector;
    }
};

int main(int, char *[])
{
  typedef itk::VectorImage<float, 2>  ImageType;

  ImageType::RegionType region;
  ImageType::IndexType start;
  start[0] = 0;
  start[1] = 0;

  ImageType::SizeType size;
  size[0] = 2;
  size[1] = 3;

  region.SetSize(size);
  region.SetIndex(start);

  ImageType::Pointer image = ImageType::New();
  image->SetRegions(region);
  image->SetVectorLength(2);
  image->Allocate();

  ImageType::IndexType pixelIndex;
  pixelIndex[0] = 1;
  pixelIndex[1] = 1;

  typedef itk::VariableLengthVector<float> VectorType;
  VectorType v;
  v.SetSize(2);
  v[0] = 1;
  v[1] = 0;

  image->SetPixel(pixelIndex, v);

  typedef itk::UnaryFunctorImageFilter<ImageType,ImageType,
                                  RotateVectors<
    ImageType::PixelType,
    ImageType::PixelType> > FilterType;

  FilterType::Pointer filter = FilterType::New();
  filter->SetInput(image);
  filter->Update();

  ImageType::PixelType inputPixelValue = image->GetPixel(pixelIndex);
  ImageType::PixelType outputPixelValue = filter->GetOutput()->GetPixel(pixelIndex);

  std::cout << "pixel (1,1) was = " << inputPixelValue << std::endl;
  std::cout << "pixel (1,1) now = " << outputPixelValue << std::endl;

  return EXIT_SUCCESS;
}