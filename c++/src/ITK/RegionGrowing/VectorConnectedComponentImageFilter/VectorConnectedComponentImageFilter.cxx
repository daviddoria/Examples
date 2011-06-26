#include "itkImage.h"

//#include "itkCovariantVector.h"
#include "itkVector.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkVectorConnectedComponentImageFilter.h"

// works
typedef itk::Image< itk::CovariantVector<unsigned char,3>, 2 >  InputImageType;

// works
//typedef itk::Image< itk::Vector<unsigned char,3>, 2 >  InputImageType;

// Does not work
//#include "itkVectorImage.h"
//typedef itk::VectorImage< unsigned char, 2 >  InputImageType;

// Does not work
//typedef itk::Image< unsigned char, 2 >  InputImageType;


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
  
  // Read input image
  typedef itk::ImageFileReader<InputImageType> ReaderType;
  ReaderType::Pointer reader = ReaderType::New();
  reader->SetFileName(inputFileName.c_str());
  reader->Update();

  // Perform the connected component analysis
  typedef itk::VectorConnectedComponentImageFilter<InputImageType, OutputImageType> ConnectedComponentFilterType;
  ConnectedComponentFilterType::Pointer connectedComponentFilter = ConnectedComponentFilterType::New();
  connectedComponentFilter->SetDistanceThreshold(.01);
  connectedComponentFilter->SetInput(reader->GetOutput());
  connectedComponentFilter->Update();

  // Write output
  typedef  itk::ImageFileWriter< OutputImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName(outputFileName);
  writer->SetInput(connectedComponentFilter->GetOutput());
  writer->Update();
  
  return EXIT_SUCCESS;
}
