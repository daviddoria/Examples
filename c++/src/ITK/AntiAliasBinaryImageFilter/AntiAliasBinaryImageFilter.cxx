#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkAntiAliasBinaryImageFilter.h"

#include "QuickView.h"

typedef itk::Image<unsigned char, 2>  UnsignedCharImageType;
typedef itk::Image<float, 2>  FloatImageType;

static void CreateImage(UnsignedCharImageType::Pointer image);

int main(int, char *[])
{
  UnsignedCharImageType::Pointer image = UnsignedCharImageType::New();
  CreateImage(image);

  // Take the absolute value of the image
  typedef itk::AntiAliasBinaryImageFilter <UnsignedCharImageType, FloatImageType>
          AntiAliasBinaryImageFilterType;

  AntiAliasBinaryImageFilterType::Pointer antiAliasFilter
          = AntiAliasBinaryImageFilterType::New ();
  antiAliasFilter->SetInput(image);

  QuickView viewer;
  viewer.AddImage<UnsignedCharImageType>(image);
  viewer.AddImage<FloatImageType>(antiAliasFilter->GetOutput());
  viewer.Visualize();

  return EXIT_SUCCESS;
}

void CreateImage(UnsignedCharImageType::Pointer image)
{
  UnsignedCharImageType::IndexType start;
  start.Fill(0);

  UnsignedCharImageType::SizeType size;
  size.Fill(200);

  UnsignedCharImageType::RegionType region;
  region.SetSize(size);
  region.SetIndex(start);

  image->SetRegions(region);
  image->Allocate();

  itk::ImageRegionIterator<UnsignedCharImageType> imageIterator(image,region);

  while(!imageIterator.IsAtEnd())
    {
    if(imageIterator.GetIndex()[0] - imageIterator.GetIndex()[1] > 5) // some pixels white, some black
      {
      imageIterator.Set(255);
      }
    else
      {
      imageIterator.Set(0);
      }
    ++imageIterator;
    }

}