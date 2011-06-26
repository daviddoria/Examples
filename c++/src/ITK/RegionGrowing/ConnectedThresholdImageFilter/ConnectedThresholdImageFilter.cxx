#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkConnectedThresholdImageFilter.h"

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

  typedef itk::ConnectedThresholdImageFilter<ImageType, ImageType> ConnectedFilterType;
  ConnectedFilterType::Pointer connectedThreshold = ConnectedFilterType::New();
  connectedThreshold->SetLower(100);
  connectedThreshold->SetUpper(200);
  connectedThreshold->SetReplaceValue(255);

  // Set seed
  ImageType::IndexType seed;
  seed[0] = 60;
  seed[1] = 70;
  connectedThreshold->SetSeed(seed);
  connectedThreshold->SetInput(reader->GetOutput());
  connectedThreshold->Update();

  typedef  itk::ImageFileWriter< ImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName(outputFileName);
  writer->SetInput(connectedThreshold->GetOutput());
  writer->Update();
  
  return EXIT_SUCCESS;
}
