#include "itkVectorImage.h"
#include "itkVector.h"
#include "itkVariableLengthVector.h"
#include "itkRigid2DTransform.h"
#include "itkBinaryFunctorImageFilter.h"

typedef itk::Image<float, 2>  ImageType;
static void CreateImage(ImageType::Pointer image);

namespace Functor
{
template< class TPixel>
class MySquaredDifference
{
public:
  MySquaredDifference() {}
  ~MySquaredDifference() {}
  bool operator!=(const MySquaredDifference &) const
  {
    return false;
  }

  bool operator==(const MySquaredDifference & other) const
  {
    return !( *this != other );
  }

  inline TPixel operator()(const TPixel & A,
                            const TPixel & B) const
  {
    const double dA = static_cast< double >( A );
    const double dB = static_cast< double >( B );
    const double diff = dA - dB;

    return static_cast< TPixel >( diff * diff );
  }
};
}

int main(int, char *[])
{
  ImageType::Pointer image1 = ImageType::New();
  CreateImage(image1);
  image1->FillBuffer(2);

  ImageType::Pointer image2 = ImageType::New();
  CreateImage(image2);
  image2->FillBuffer(5);

  typedef itk::BinaryFunctorImageFilter< ImageType, ImageType, ImageType,
                            Functor::MySquaredDifference<ImageType::PixelType> > FilterType;

  FilterType::Pointer filter = FilterType::New();
  filter->SetInput1(image1);
  filter->SetInput2(image2);
  filter->Update();

  itk::Index<2> pixelIndex;
  pixelIndex.Fill(0);

  ImageType::PixelType input1PixelValue = image1->GetPixel(pixelIndex);
  ImageType::PixelType input2PixelValue = image2->GetPixel(pixelIndex);
  ImageType::PixelType outputPixelValue = filter->GetOutput()->GetPixel(pixelIndex);

  std::cout << "pixel1 was = " << input1PixelValue << std::endl;
  std::cout << "pixel2 was = " << input2PixelValue << std::endl;
  std::cout << "output is = " << outputPixelValue << std::endl;

  return EXIT_SUCCESS;
}

void CreateImage(ImageType::Pointer image)
{
  ImageType::IndexType start;
  start.Fill(0);

  ImageType::SizeType size;
  size.Fill(10);

  ImageType::RegionType region(start,size);
  image->SetRegions(region);
  image->Allocate();

}