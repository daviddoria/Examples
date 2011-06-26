#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"

#include "ImageFilter.h"

int main(int, char*[])
{
  // Setup types
  typedef itk::Image<unsigned char, 2>   ImageType;
  typedef itk::ImageFilter<ImageType>  FilterType;

  // Create and the filter
  FilterType::Pointer filter = FilterType::New();
  filter->Update();

  return EXIT_SUCCESS;
}