#include "itkImage.h"
#include <itkImageFileReader.h>
#include <itkExtractImageFilter.h>

#include <iostream>
#include <string>

int main(int argc, char *argv[])
{
  typedef itk::Image<unsigned char, 2> ImageType;

  ImageType::IndexType start;
  start.Fill(0);

  ImageType::SizeType size;
  size.Fill(10);

  ImageType::RegionType region(start, size);
  
  ImageType::Pointer image = ImageType::New();
  image->SetRegions(region);
  image->Allocate();
  image->FillBuffer(5);

  std::cout << "Image largest region: " << image->GetLargestPossibleRegion() << std::endl;

  //////////////
  ImageType::IndexType desiredStart;
  desiredStart.Fill(3);

  ImageType::SizeType desiredSize;
  desiredSize.Fill(4);

  ImageType::RegionType desiredRegion(desiredStart, desiredSize);

  std::cout << "desiredRegion: " << desiredRegion << std::endl;
  
  typedef itk::ExtractImageFilter< ImageType, ImageType > FilterType;
  FilterType::Pointer filter = FilterType::New();
  filter->SetExtractionRegion(desiredRegion);
  filter->SetInput(image);
  filter->SetDirectionCollapseToIdentity();
  filter->Update();
  
  ImageType::Pointer output = filter->GetOutput();
  output->DisconnectPipeline();
  output->FillBuffer(2);

  itk::Index<2> index;
  index.Fill(5);

  std::cout << "new largest region: " << output->GetLargestPossibleRegion() << std::endl;
  std::cout << "new: " << (int)output->GetPixel(index) << std::endl;
  std::cout << "Original: " << (int)image->GetPixel(index) << std::endl;

  return EXIT_SUCCESS;
}