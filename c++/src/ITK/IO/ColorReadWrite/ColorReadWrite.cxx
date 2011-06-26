#include "itkImage.h"
#include "itkCovariantVector.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"

typedef itk::CovariantVector<unsigned char,3> ColorPixelType;
typedef itk::Image<ColorPixelType, 2> ColorImageType;

int main(int argc, char *argv[])
{
  if(argc < 2)
    {
    std::cerr << "Required: filename" << std::endl;

    return EXIT_FAILURE;
    }
  std::string inputFilename = argv[1];

  typedef itk::ImageFileReader<ColorImageType> ReaderType;
  ReaderType::Pointer reader = ReaderType::New();
  reader->SetFileName(inputFilename.c_str());
  reader->Update();

  typedef itk::ImageFileWriter<ColorImageType> WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetInput(reader->GetOutput());
  writer->SetFileName("output.png");
  writer->Update();

  return EXIT_SUCCESS;
}