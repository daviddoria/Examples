#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkConvolutionImageFilter.h"
#include "itkImageRegionIterator.h"
#include "itkCovariantVector.h"

typedef itk::Image<itk::CovariantVector<unsigned char, 3>, 2> ImageType;

void CreateKernel(ImageType::Pointer kernel);

int main(int argc, char * argv[])
{
  // Verify command line arguments
  if( argc < 2 )
    {
      std::cerr << "Usage: " << std::endl;
      std::cerr << argv[0] << "inputImageFile" << std::endl;
      return EXIT_FAILURE;
    }

  // Parse command line arguments
  std::string inputFilename = argv[1];

  ImageType::Pointer kernel = ImageType::New();
  CreateKernel(kernel);

  typedef itk::ImageFileReader<ImageType>  readerType;

  typedef itk::ConvolutionImageFilter<ImageType, ImageType> FilterType;

  // Create and setup a reader
  readerType::Pointer reader = readerType::New();
  reader->SetFileName( inputFilename.c_str() );

  // Convolve image with kernel.
  FilterType::Pointer convolutionFilter = FilterType::New();
  convolutionFilter->SetInput(reader->GetOutput());
  convolutionFilter->SetImageKernelInput(kernel);
  convolutionFilter->Update();

  typedef  itk::ImageFileWriter< ImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("TestOutput.jpg");
  writer->SetInput(convolutionFilter->GetOutput());
  writer->Update();

  return EXIT_SUCCESS;
}

void CreateKernel(ImageType::Pointer kernel)
{
  ImageType::IndexType start;
  start.Fill(0);

  ImageType::SizeType size;
  size.Fill(3);

  ImageType::RegionType region;
  region.SetSize(size);
  region.SetIndex(start);

  kernel->SetRegions(region);
  kernel->Allocate();

  itk::ImageRegionIterator<ImageType> imageIterator(kernel, region);
  imageIterator.GoToBegin();

  itk::CovariantVector<unsigned char, 3> blackPixel;
  blackPixel.Fill(0);

  while(!imageIterator.IsAtEnd())
    {
    imageIterator.Set(blackPixel);

    ++imageIterator;
    }
}