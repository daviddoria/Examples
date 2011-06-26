#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkVectorRescaleIntensityImageFilter.h"
#include "itkCastImageFilter.h"

int main(int argc, char *argv[])
{
  if(argc < 3)
    {
    std::cerr << "Required: input output" << std::endl;
    return EXIT_FAILURE;
    }

  std::string inputFilename = argv[1];
  std::string outputFilename = argv[2];

  typedef itk::Image<itk::CovariantVector<float, 3>, 2>  FloatImageType;
  typedef itk::Image<itk::CovariantVector<unsigned char, 3>, 2>  UnsignedCharImageType;

  typedef itk::ImageFileReader<FloatImageType> ReaderType;
  ReaderType::Pointer reader = ReaderType::New();
  reader->SetFileName(inputFilename);
  reader->Update();

  typedef itk::VectorRescaleIntensityImageFilter<FloatImageType, UnsignedCharImageType> VectorRescaleFilterType;
  VectorRescaleFilterType::Pointer rescaleFilter = VectorRescaleFilterType::New();
  rescaleFilter->SetInput(reader->GetOutput());
  rescaleFilter->SetOutputMaximumMagnitude(255);
  rescaleFilter->Update();

  typedef  itk::ImageFileWriter<UnsignedCharImageType> WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName(outputFilename);
  writer->SetInput(rescaleFilter->GetOutput());
  writer->Update();

  return EXIT_SUCCESS;
}