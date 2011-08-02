#include "itkVectorImage.h"
#include "itkImageRegionIteratorWithIndex.h"
#include "itkImageFileWriter.h"

int main(int, char *[])
{
  // Create an image
  typedef itk::VectorImage<unsigned char, 2>  ImageType;

  ImageType::IndexType start;
  start.Fill(0);

  ImageType::SizeType size;
  size.Fill(100);

  ImageType::RegionType region(start,size);

  ImageType::Pointer image = ImageType::New();
  image->SetRegions(region);
  image->SetVectorLength(3);
  image->Allocate();
  
  itk::ImageRegionIteratorWithIndex< ImageType > it( image, region );

  // Fill the image with some color pattern
  
  ImageType::PixelType pixelValue;
  pixelValue.SetSize(3);
  
  it.GoToBegin();
  while( !it.IsAtEnd() )
    {
    ImageType::IndexType index = it.GetIndex();
    pixelValue[0] =  index[0] * 2;
    pixelValue[1] =  index[0] + index[1];
    pixelValue[2] =  index[1] * 2;
    it.Set( pixelValue );
    ++it;
    }

  // Write an image for regression testing
  typedef itk::ImageFileWriter<  ImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetInput (image);
  writer->SetFileName("output.png");
  writer->Update();
  
  return EXIT_SUCCESS;
}
