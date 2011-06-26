#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"

#include "ImageFilterMultipleOutputs.h"

int main(int, char*[])
{
  // Setup types
  typedef itk::Image<unsigned char, 2>   ImageType;
  typedef itk::ImageFilterMultipleOutputs<ImageType>  FilterType;

  // Create and the filter
  FilterType::Pointer filter = FilterType::New();
  filter->Update();

  {
  typedef  itk::ImageFileWriter< ImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("TestOutput1.jpg");
  writer->SetInput(filter->GetOutput1());
  writer->Update();
  }
  
  {
  typedef  itk::ImageFileWriter< ImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("TestOutput2.jpg");
  writer->SetInput(filter->GetOutput2());
  writer->Update();
  }
  
  return EXIT_SUCCESS;
}