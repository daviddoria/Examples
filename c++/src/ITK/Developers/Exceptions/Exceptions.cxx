#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"

#include "ImageFilter.h"

int main(int, char*[])
{
  // Setup types
  typedef itk::Image<unsigned char, 2>   ImageType;
  ImageType::Pointer image = ImageType::New();
  
  // Create and the filter
  typedef itk::ImageFilter<ImageType>  FilterType;
  FilterType::Pointer filter = FilterType::New();
  filter->SetInput(image);
  filter->Update();

  return EXIT_SUCCESS;
}