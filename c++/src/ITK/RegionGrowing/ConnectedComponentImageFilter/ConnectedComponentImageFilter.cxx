#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkConnectedComponentImageFilter.h"

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

  typedef itk::ConnectedComponentImageFilter<ImageType, ImageType> ConnectedComponentFilterType;
  ConnectedComponentFilterType::Pointer connectedComponentFilter = ConnectedComponentFilterType::New();

  // Set seed
  itk::Index<2> seed;
  seed[0] = 60;
  seed[1] = 70;
  connectedComponentFilter->SetInput(reader->GetOutput());
  connectedComponentFilter->Update();

  typedef  itk::ImageFileWriter< ImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName(outputFileName);
  writer->SetInput(connectedComponentFilter->GetOutput());
  writer->Update();
  
  return EXIT_SUCCESS;
}
