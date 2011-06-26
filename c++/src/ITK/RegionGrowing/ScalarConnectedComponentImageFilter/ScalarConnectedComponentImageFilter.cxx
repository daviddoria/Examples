#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkScalarConnectedComponentImageFilter.h"

typedef itk::Image< unsigned char, 2 >  ImageType;

int main( int argc, char *argv[])
{
  if(argc < 3)
    {
    std::cerr << "Required: filename.png output.png" << std::endl;

    return EXIT_FAILURE;
    }
  std::string inputFileName = argv[1];
  std::string outputFileName = argv[2];
  
  typedef itk::ImageFileReader<ImageType> ReaderType;
  ReaderType::Pointer reader = ReaderType::New();
  reader->SetFileName(inputFileName.c_str());
  reader->Update();

  typedef itk::ScalarConnectedComponentImageFilter<ImageType, ImageType> ScalarConnectedComponentImageFilterType;
  ScalarConnectedComponentImageFilterType::Pointer scalarConnectedComponentFilter = ScalarConnectedComponentImageFilterType::New();
  scalarConnectedComponentFilter->SetInput(reader->GetOutput());
  scalarConnectedComponentFilter->Update();

  typedef  itk::ImageFileWriter< ImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName(outputFileName);
  writer->SetInput(connectedThreshold->GetOutput());
  writer->Update();
  
  return EXIT_SUCCESS;
}
