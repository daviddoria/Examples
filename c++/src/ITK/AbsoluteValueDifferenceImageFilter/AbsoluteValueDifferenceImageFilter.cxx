#include "itkImage.h"
#include "itkAbsoluteValueDifferenceImageFilter.h"
#include "itkImageRegionIterator.h"

typedef itk::Image<unsigned char, 2>  UnsignedCharImageType;
typedef itk::Image<float, 2>  FloatImageType;

static void CreateImage1(UnsignedCharImageType::Pointer image);
static void CreateImage2(UnsignedCharImageType::Pointer image);

int main(int, char *[])
{
  UnsignedCharImageType::Pointer image1 = UnsignedCharImageType::New();
  CreateImage1(image1);

  UnsignedCharImageType::Pointer image2 = UnsignedCharImageType::New();
  CreateImage2(image2);

  typedef itk::AbsoluteValueDifferenceImageFilter <UnsignedCharImageType, UnsignedCharImageType,
                                             FloatImageType>
          AbsoluteValueDifferenceImageFilterType;

  AbsoluteValueDifferenceImageFilterType::Pointer absoluteValueDifferenceFilter
          = AbsoluteValueDifferenceImageFilterType::New ();
  absoluteValueDifferenceFilter->SetInput1(image1);
  absoluteValueDifferenceFilter->SetInput2(image2);
  absoluteValueDifferenceFilter->Update();

  return EXIT_SUCCESS;
}

void CreateImage1(UnsignedCharImageType::Pointer image)
{
  UnsignedCharImageType::IndexType start;
  start.Fill(0);

  UnsignedCharImageType::SizeType size;
  size.Fill(10);

  UnsignedCharImageType::RegionType region;
  region.SetSize(size);
  region.SetIndex(start);

  image->SetRegions(region);
  image->Allocate();

  itk::ImageRegionIterator<UnsignedCharImageType> imageIterator(image,region);

  while(!imageIterator.IsAtEnd())
    {
    imageIterator.Set(255);

    ++imageIterator;
    }
}


void CreateImage2(UnsignedCharImageType::Pointer image)
{
  // Create an image with 2 connected components
  UnsignedCharImageType::IndexType start;
  start.Fill(0);

  UnsignedCharImageType::SizeType size;
  size.Fill(10);

  UnsignedCharImageType::RegionType region;
  region.SetSize(size);
  region.SetIndex(start);

  image->SetRegions(region);
  image->Allocate();

  itk::ImageRegionIterator<UnsignedCharImageType> imageIterator(image,region);

  while(!imageIterator.IsAtEnd())
    {
    imageIterator.Set(100);

    ++imageIterator;
    }

}
