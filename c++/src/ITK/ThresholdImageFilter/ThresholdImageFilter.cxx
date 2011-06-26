#include "itkImage.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkThresholdImageFilter.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"


typedef itk::Image<unsigned char, 2>  ImageType;

int main(int argc, char *argv[])
{
  if(argc < 3)
    {
    std::cerr << "Required: inputFileName.png outputFileName.png" << std::endl;
    return EXIT_FAILURE;
    }

  std::string inputFileName = argv[1];
  std::string outputFileName = argv[2];
  
  typedef itk::ImageFileReader<ImageType> ReaderType;
  ReaderType::Pointer reader = ReaderType::New();
  reader->SetFileName(inputFileName);
  reader->Update();

  typedef itk::ThresholdImageFilter <ImageType>
          ThresholdImageFilterType;

  ThresholdImageFilterType::Pointer thresholdFilter
          = ThresholdImageFilterType::New();
  thresholdFilter->SetInput(reader->GetOutput());
  thresholdFilter->ThresholdBelow(200);
  thresholdFilter->SetOutsideValue(0);
  thresholdFilter->Update();

  typedef itk::ImageFileWriter<ImageType> WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName(outputFileName);
  writer->SetInput(thresholdFilter->GetOutput());
  writer->Update();

  return EXIT_SUCCESS;
}
