#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkConstantPadImageFilter.h"
#include "itkImageRegionIterator.h"

typedef itk::Image<unsigned char, 2>  ImageType;

void CreateImage(ImageType::Pointer image);
void WriteImage(ImageType::Pointer image, std::string filename);

int main(int, char *[])
{
  ImageType::Pointer image = ImageType::New();
  CreateImage(image);
  WriteImage(image, "input.png");

  typedef itk::ConstantPadImageFilter <ImageType, ImageType>
          ConstantPadImageFilterType;

  ImageType::SizeType lowerExtendRegion;
  lowerExtendRegion[0] = 10;
  lowerExtendRegion[1] = 20;

  ImageType::SizeType upperExtendRegion;
  upperExtendRegion[0] = 50;
  upperExtendRegion[1] = 30;

  ImageType::PixelType constantPixel = 100;

  ConstantPadImageFilterType::Pointer padFilter
          = ConstantPadImageFilterType::New();
  padFilter->SetInput(image);
  //padFilter->SetPadBound(outputRegion); // Calls SetPadLowerBound(region) and SetPadUpperBound(region)
  padFilter->SetPadLowerBound(lowerExtendRegion);
  padFilter->SetPadUpperBound(upperExtendRegion);
  padFilter->SetConstant(constantPixel);
  padFilter->Update();

  WriteImage(padFilter->GetOutput(), "output.png");

  return EXIT_SUCCESS;
}

void CreateImage(ImageType::Pointer image)
{
  // Create an image
  ImageType::IndexType start;
  start.Fill(0);

  ImageType::SizeType size;
  size.Fill(100);

  ImageType::RegionType region(start, size);

  image->SetRegions(region);
  image->Allocate();
  image->FillBuffer(0);

}

void WriteImage(ImageType::Pointer image, std::string filename)
{
  typedef  itk::ImageFileWriter< ImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName(filename);
  writer->SetInput(image);
  writer->Update();
}