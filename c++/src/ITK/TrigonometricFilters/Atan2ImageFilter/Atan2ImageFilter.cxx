#include "itkAtan2ImageFilter.h"
#include "itkImage.h"

typedef itk::Image<float, 2>  FloatImageType;

static void CreateImage1(FloatImageType::Pointer image);
static void CreateImage2(FloatImageType::Pointer image);

int main(int, char *[])
{
  FloatImageType::Pointer image1 = FloatImageType::New();
  CreateImage1(image1);

  FloatImageType::Pointer image2 = FloatImageType::New();
  CreateImage2(image2);

  // Compute the atan of each pixel
  typedef itk::Atan2ImageFilter <FloatImageType, FloatImageType, FloatImageType>
          Atan2ImageFilterType;

  Atan2ImageFilterType::Pointer atan2ImageFilter
          = Atan2ImageFilterType::New ();
  atan2ImageFilter->SetInput1(image1);
  atan2ImageFilter->SetInput2(image2);
  atan2ImageFilter->Update();

  return EXIT_SUCCESS;
}

void CreateImage1(FloatImageType::Pointer image)
{
  FloatImageType::IndexType start;
  start.Fill(0);

  FloatImageType::SizeType size;
  size.Fill(10);

  FloatImageType::RegionType region(start,size);

  image->SetRegions(region);
  image->Allocate();

  itk::ImageRegionIterator<FloatImageType> imageIterator(image,region);

  while(!imageIterator.IsAtEnd())
    {
    imageIterator.Set(imageIterator.GetIndex()[0] - imageIterator.GetIndex()[1]);
    ++imageIterator;
    }

}

void CreateImage2(FloatImageType::Pointer image)
{
  FloatImageType::IndexType start;
  start.Fill(0);

  FloatImageType::SizeType size;
  size.Fill(10);

  FloatImageType::RegionType region(start,size);

  image->SetRegions(region);
  image->Allocate();

  itk::ImageRegionIterator<FloatImageType> imageIterator(image,region);

  while(!imageIterator.IsAtEnd())
    {
    imageIterator.Set(imageIterator.GetIndex()[0] - imageIterator.GetIndex()[1]);
    ++imageIterator;
    }

}
