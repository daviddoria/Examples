#include "itkImage.h"
#include "itkVector.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkVectorConfidenceConnectedImageFilter.h"

typedef itk::Image< itk::Vector<unsigned char, 3>, 2 >  InputImageType;
typedef itk::Image< unsigned char, 2 >  OutputImageType;

int main( int argc, char *argv[])
{
  if(argc < 3)
    {
    std::cerr << "Required: filename.png output.png" << std::endl;

    return EXIT_FAILURE;
    }
  std::string inputFileName = argv[1];
  std::string outputFileName = argv[2];
  
  typedef itk::ImageFileReader<InputImageType> ReaderType;
  ReaderType::Pointer reader = ReaderType::New();
  reader->SetFileName(inputFileName.c_str());
  reader->Update();

  typedef itk::VectorConfidenceConnectedImageFilter<InputImageType, OutputImageType> VectorConfidenceConnectedFilterType;
  VectorConfidenceConnectedFilterType::Pointer vectorConfidenceConnectedFilter = VectorConfidenceConnectedFilterType::New();
  vectorConfidenceConnectedFilter->SetNumberOfIterations(100);
  vectorConfidenceConnectedFilter->SetMultiplier(2);
  vectorConfidenceConnectedFilter->SetReplaceValue(255);

  // Set seed
  itk::Index<2> seed;
  seed[0] = 60;
  seed[1] = 70;
  vectorConfidenceConnectedFilter->SetSeed(seed);
  vectorConfidenceConnectedFilter->SetInput(reader->GetOutput());
  vectorConfidenceConnectedFilter->Update();

  typedef  itk::ImageFileWriter< OutputImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName(outputFileName);
  writer->SetInput(vectorConfidenceConnectedFilter->GetOutput());
  writer->Update();
  
  return EXIT_SUCCESS;
}
