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
    outputFilename = "/home/doriad/test.tif";
    }

  typedef itk::RGBAPixel<unsigned char> PixelType;
  typedef itk::Image< PixelType, 2>  ImageType;

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
    ImageType::PixelType pixel = imageIterator.Get();
  
    if(imageIterator.GetIndex()[0] > 100)
      {
      pixel.SetRed(0);
      pixel.SetGreen(255);
      pixel.SetBlue(0);
      //pixel.SetAlpha(255); // invisible
      pixel.SetAlpha(122);
      }
    else
      {
      pixel.SetRed(255);
      pixel.SetGreen(0);
      pixel.SetBlue(0);
      pixel.SetAlpha(0.5);
      }
    imageIterator.Set(pixel);
    
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