#include "itkImage.h"
#include "itkImageFileWriter.h"

int main(int argc, char *argv[])
{
  std::string outputFilename;
  if(argc > 1)
    {
    outputFilename = argv[1];
    }
  else
    {
    outputFilename = "test.png";
    }

  typedef unsigned char     PixelType;
  const     unsigned int    Dimension = 2;
  typedef itk::Image< PixelType, Dimension >  ImageType;


  ImageType::IndexType start;
  start.Fill(0);

  ImageType::SizeType size;
  size.Fill(100);

  ImageType::RegionType region(start, size);

  ImageType::Pointer image = ImageType::New();
  image->SetRegions(region);
  image->Allocate();

  ImageType::IndexType ind;
  ind[0] = 10;
  ind[1] = 10;

  typedef  itk::ImageFileWriter< ImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName(outputFilename);
  writer->SetInput(image);
  writer->Update();

  return EXIT_SUCCESS;
}
