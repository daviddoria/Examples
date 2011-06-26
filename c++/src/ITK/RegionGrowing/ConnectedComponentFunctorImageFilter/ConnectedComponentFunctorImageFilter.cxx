#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkConnectedComponentFunctorImageFilter.h"

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

  typedef itk::ConnectedComponentFunctorImageFilter<ImageType, ImageType> ConnectedComponentFunctorImageFilterType;
  ConnectedComponentFunctorImageFilterType::Pointer connectedComponentFunctorFilter = ConnectedComponentFunctorImageFilterType::New();
  //connectedComponentFunctorFilter->SetFunctor();
  connectedComponentFunctorFilter->SetReplaceValue(255);

  // Set seed
  itk::Index<2> seed;
  seed[0] = 60;
  seed[1] = 70;
  connectedComponentFunctorFilter->SetSeed(seed);
  connectedComponentFunctorFilter->SetInput(reader->GetOutput());
  connectedComponentFunctorFilter->Update();

  typedef  itk::ImageFileWriter< ImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName(outputFileName);
  writer->SetInput(connectedThreshold->GetOutput());
  writer->Update();
  
  return EXIT_SUCCESS;
}
