#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkImageRegionIterator.h"
#include "itkTIFFImageIO.h"
#include "itkRGBAPixel.h"

int main(int argc, char *argv[])
{
  std::string outputFilename;
  if(argc > 1)
    {
    outputFilename = argv[1];
    }
  else
    {
    outputFilename = "test.tif";
    }

  typedef itk::Image< unsigned char, 2>  ImageType;

  ImageType::RegionType region;
  ImageType::IndexType start;
  start[0] = 0;
  start[1] = 0;

  ImageType::SizeType size;
  size[0] = 200;
  size[1] = 300;

  region.SetSize(size);
  region.SetIndex(start);

  ImageType::Pointer image = ImageType::New();
  image->SetRegions(region);
  image->Allocate();

  itk::ImageRegionIterator<ImageType> imageIterator(image,region);

  while(!imageIterator.IsAtEnd())
    {
    if(imageIterator.GetIndex()[0] > 100)
      {
      imageIterator.Set(100);
      }
    else
      {
      imageIterator.Set(200);
      }

    ++imageIterator;
  }

  typedef  itk::ImageFileWriter< ImageType  > WriterType;
  typedef  itk::TIFFImageIO TIFFIOType;
  WriterType::Pointer writer = WriterType::New();
  TIFFIOType::Pointer tiffIO = TIFFIOType::New();
  tiffIO->SetPixelType(itk::ImageIOBase::RGBA);
  writer->SetFileName(outputFilename);
  writer->SetInput(image);
  writer->SetImageIO(tiffIO);
  writer->Update();

  return EXIT_SUCCESS;
}