#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"

int main(int argc, char *argv[])
{
  if(argc < 3)
    {
    std::cerr << "Required: input output" << std::endl;

    return EXIT_FAILURE;
    }
  std::string inputFilename = argv[1];
  std::string outputFilename = argv[2];

  typedef itk::Image< unsigned char, 2 >  ImageType;

  typedef itk::ImageFileReader<ImageType> ReaderType;
  ReaderType::Pointer reader = ReaderType::New();
  reader->SetFileName(inputFilename.c_str());
  reader->Update();

  typedef itk::ImageFileWriter<ImageType> WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetInput(reader->GetOutput());
  writer->SetFileName(outputFilename);
  writer->Update();

  return EXIT_SUCCESS;
}
