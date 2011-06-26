#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkConvolutionImageFilter.h"
#include "itkImageRegionIterator.h"

typedef itk::Image<unsigned char, 2> ImageType;
typedef itk::Image<float, 2> KernelType;

void CreateImage(ImageType::Pointer kernel);
void CreateKernel(KernelType::Pointer kernel);

int main(int, char*[])
{
  ImageType::Pointer image = ImageType::New();
  CreateImage(image);

  KernelType::Pointer kernel = KernelType::New();
  CreateKernel(kernel);

  //typedef itk::ConvolutionImageFilter<ImageType> FilterType;
  typedef itk::ConvolutionImageFilter<ImageType, ImageType, KernelType> FilterType;

  // Convolve image with kernel.
  FilterType::Pointer convolutionFilter = FilterType::New();
  convolutionFilter->SetInput(image);
  convolutionFilter->SetImageKernelInput(kernel);
  convolutionFilter->Update();

  return EXIT_SUCCESS;
}

void CreateKernel(KernelType::Pointer kernel)
{
  KernelType::IndexType start;
  start.Fill(0);

  KernelType::SizeType size;
  size.Fill(3);

  KernelType::RegionType region;
  region.SetSize(size);
  region.SetIndex(start);

  kernel->SetRegions(region);
  kernel->Allocate();

  itk::ImageRegionIterator<KernelType> imageIterator(kernel, region);

  while(!imageIterator.IsAtEnd())
    {
    imageIterator.Set(1);
    ++imageIterator;
    }
}


void CreateImage(ImageType::Pointer image)
{
  ImageType::IndexType start;
  start.Fill(0);

  ImageType::SizeType size;
  size.Fill(3);

  ImageType::RegionType region;
  region.SetSize(size);
  region.SetIndex(start);

  image->SetRegions(region);
  image->Allocate();

  itk::ImageRegionIterator<ImageType> imageIterator(image, region);

  while(!imageIterator.IsAtEnd())
    {
    imageIterator.Set(1);
    ++imageIterator;
    }
}