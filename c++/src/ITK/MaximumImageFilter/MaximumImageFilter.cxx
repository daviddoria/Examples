#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkMaximumImageFilter.h"
#include "itkImageFileReader.h"
#include "itkImageRegionIterator.h"

typedef itk::Image<unsigned char, 2>  ImageType;

void CreateImage1(ImageType* image);
void CreateImage2(ImageType* image);

int main(int, char*[])
{
  ImageType::Pointer image1 = ImageType::New();
  CreateImage1(image1);

  ImageType::Pointer image2 = ImageType::New();
  CreateImage2(image2);

  typedef itk::MaximumImageFilter <ImageType>
          MaximumImageFilterType;

  MaximumImageFilterType::Pointer maximumImageFilter
          = MaximumImageFilterType::New ();
  maximumImageFilter->SetInput(0, image1);
  maximumImageFilter->SetInput(1, image2);
  maximumImageFilter->Update();

  return EXIT_SUCCESS;
}

void CreateImage1(ImageType* image)
{
  ImageType::IndexType start;
  start.Fill(0);

  ImageType::SizeType size;
  size.Fill(100);

  ImageType::RegionType region;
  region.SetSize(size);
  region.SetIndex(start);

  image->SetRegions(region);
  image->Allocate();

  itk::ImageRegionIterator<ImageType> imageIterator(image,region);

  while(!imageIterator.IsAtEnd())
    {
    if(imageIterator.GetIndex()[0] < 30)
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

void CreateImage2(ImageType* image)
{
  ImageType::IndexType start;
  start.Fill(0);

  ImageType::SizeType size;
  size.Fill(100);

  ImageType::RegionType region;
  region.SetSize(size);
  region.SetIndex(start);

  image->SetRegions(region);
  image->Allocate();

  itk::ImageRegionIterator<ImageType> imageIterator(image,region);

  while(!imageIterator.IsAtEnd())
    {
    if(imageIterator.GetIndex()[0] > 70)
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