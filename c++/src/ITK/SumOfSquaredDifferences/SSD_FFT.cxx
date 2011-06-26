#include "itkImage.h"
#include "itkImageRegionIterator.h"

typedef itk::Image<float, 2> ImageType;

void CreateBigImage(ImageType::Pointer image);
void CreateSmallImage(ImageType::Pointer image);

int main(int, char *[])
{
  ImageType::Pointer image = ImageType::New();
  CreateBigImage(image);

  ImageType::Pointer smallImage = ImageType::New();
  CreateSmallImage(smallImage);

  return EXIT_SUCCESS;
}

void CreateBigImage(ImageType::Pointer image)
{
  ImageType::IndexType start;
  start.Fill(0);

  ImageType::SizeType size;
  size.Fill(100);

  ImageType::RegionType region(start, size);
  region.SetSize(size);
  region.SetIndex(start);

  image->SetRegions(region);
  image->Allocate();

  itk::ImageRegionIterator<ImageType> imageIterator(image,region);

  while(!imageIterator.IsAtEnd())
    {
    imageIterator.Set(255);

    ++imageIterator;
    }
}


void CreateSmallImage(ImageType::Pointer image)
{
  // Create an image with 2 connected components
  ImageType::IndexType start;
  start.Fill(0);

  ImageType::SizeType size;
  size.Fill(10);

  ImageType::RegionType region;
  region.SetSize(size);
  region.SetIndex(start);

  image->SetRegions(region);
  image->Allocate();

  itk::ImageRegionIterator<ImageType> imageIterator(image,region);

  while(!imageIterator.IsAtEnd())
    {
    imageIterator.Set(100);

    ++imageIterator;
    }

}