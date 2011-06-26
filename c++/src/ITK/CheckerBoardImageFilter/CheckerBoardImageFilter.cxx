#include "itkImage.h"
#include "itkCheckerBoardImageFilter.h"
#include "itkImageRegionIterator.h"

#include "QuickView.h"

typedef itk::Image<unsigned char, 2> ImageType;

void CreateImage(ImageType::Pointer image, unsigned char color);

int main(int argc, char *argv[])
{
  ImageType::Pointer image1 = ImageType::New();
  CreateImage(image1, 0);
  ImageType::Pointer image2 = ImageType::New();
  CreateImage(image2, 255);

  typedef itk::CheckerBoardImageFilter< ImageType > CheckerBoardFilterType;
  CheckerBoardFilterType::Pointer checkerBoardFilter = CheckerBoardFilterType::New();
  checkerBoardFilter->SetInput1(image1);
  checkerBoardFilter->SetInput2(image2);
  checkerBoardFilter->Update();

  QuickView viewer;
  viewer.AddImage<ImageType>(image1);
  viewer.AddImage<ImageType>(image2);
  viewer.AddImage<ImageType>(checkerBoardFilter->GetOutput());
  viewer.Visualize();

  return EXIT_SUCCESS;
}

void CreateImage(ImageType::Pointer image, unsigned char color)
{
  // Create an image with all pixels set to 'color'

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
    imageIterator.Set(color);

    ++imageIterator;
    }
}