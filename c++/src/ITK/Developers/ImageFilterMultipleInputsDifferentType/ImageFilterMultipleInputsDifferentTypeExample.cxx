#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkCovariantVector.h"

#include "ImageFilterMultipleInputsDifferentType.h"

int main(int, char*[])
{
  // Setup types
  typedef itk::Image<itk::CovariantVector<unsigned char, 3>, 2>   VectorImageType;
  typedef itk::Image<unsigned char, 2> ScalarImageType;
  typedef itk::ImageFilterMultipleInputsDifferentType<VectorImageType, ScalarImageType>  FilterType;

  typedef itk::ImageFileReader<VectorImageType> ReaderType;
  ReaderType::Pointer reader = ReaderType::New();
  reader->SetFileName("Test.jpg");
  reader->Update();

  // Create and the filter
  FilterType::Pointer filter = FilterType::New();
  filter->SetInput(reader->GetOutput());
  filter->Update();

  typedef  itk::ImageFileWriter< VectorImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("TestOutput.jpg");
  writer->SetInput(filter->GetOutput());
  writer->Update();

  return EXIT_SUCCESS;
}