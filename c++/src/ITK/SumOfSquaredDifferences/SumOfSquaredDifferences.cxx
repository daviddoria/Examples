#include "itkImage.h"
#include "itkImageRegionIterator.h"
#include "itkSquaredDifferenceImageFilter.h"

typedef itk::Image<float, 2> ImageType;

void CreateBigImage(ImageType::Pointer image);
void CreateSmallImage(ImageType::Pointer image);
float PatchCompare(ImageType::Pointer image1, itk::ImageRegion<2> region1,
                   ImageType::Pointer image2, itk::ImageRegion<2> region2);

int main(int, char *[])
{
  ImageType::Pointer bigImage = ImageType::New();
  CreateBigImage(bigImage);

  ImageType::Pointer smallImage = ImageType::New();
  CreateSmallImage(smallImage);

  ImageType::Pointer outputImage = ImageType::New();
  CreateBigImage(outputImage);

  itk::ImageRegionConstIterator<ImageType> imageIterator(bigImage, bigImage->GetLargestPossibleRegion());
  itk::ImageRegionIterator<ImageType> outputIterator(outputImage, outputImage->GetLargestPossibleRegion());

  while(!imageIterator.IsAtEnd())
    {
    itk::ImageRegion<2> region(imageIterator.GetIndex(), smallImage->GetLargestPossibleRegion().GetSize());

    float ssd = PatchCompare(bigImage, region, smallImage, smallImage->GetLargestPossibleRegion());
    outputIterator.Set(ssd);

    ++imageIterator;
    ++outputIterator;
    }

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


float PatchCompare(ImageType::Pointer image1, itk::ImageRegion<2> region1,
                   ImageType::Pointer image2, itk::ImageRegion<2> region2)
{
  // sum of the normalized squared differences in the region where the two patches overlap
  itk::ImageRegionConstIterator<ImageType> patch1Iterator(image1, region1);
  itk::ImageRegionConstIterator<ImageType> patch2Iterator(image1, region2);

  double ssd = 0; // sum of squared differences
  while(!patch1Iterator.IsAtEnd())
    {
    // Get the value of the current pixel
    float pixel1 = patch1Iterator.Get();
    float pixel2 = patch2Iterator.Get();
    ssd += (pixel1 - pixel2) * (pixel1 - pixel2);

    ++patch1Iterator;
    ++patch2Iterator;
    }
  return ssd;
}