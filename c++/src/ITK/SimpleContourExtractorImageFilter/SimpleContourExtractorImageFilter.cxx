#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkSimpleContourExtractorImageFilter.h"
#include "itkImageRegionIterator.h"

typedef itk::Image<unsigned char, 2>  UnsignedCharImageType;

static void CreateImage(UnsignedCharImageType::Pointer image);

int main(int, char *[])
{
  UnsignedCharImageType::Pointer image = UnsignedCharImageType::New();
  CreateImage(image);

  typedef itk::SimpleContourExtractorImageFilter <UnsignedCharImageType, UnsignedCharImageType>
          SimpleContourExtractorImageFilterType;
  SimpleContourExtractorImageFilterType::Pointer contourFilter
          = SimpleContourExtractorImageFilterType::New();
  contourFilter->SetInput(image);
  contourFilter->Update();
   
  typedef  itk::ImageFileWriter< UnsignedCharImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("output.png");
  writer->SetInput(contourFilter->GetOutput());
  writer->Update();
  
  return EXIT_SUCCESS;
}

void CreateImage(UnsignedCharImageType::Pointer image)
{
  // Create an image
  itk::Index<2> start;
  start.Fill(0);

  itk::Size<2> size;
  size.Fill(100);

  itk::ImageRegion<2> region(start,size);

  image->SetRegions(region);
  image->Allocate();
  image->FillBuffer(0);

  // Create a line pixels
  for(unsigned int i = 40; i < 60; ++i)
    {
    itk::Index<2> pixel;
    pixel.Fill(i);
    image->SetPixel(pixel, 255);
    }
    
  // Create another line of pixels
  for(unsigned int i = 10; i < 20; ++i)
    {
    itk::Index<2> pixel;
    pixel[0] = 10;
    pixel[1] = i;
    image->SetPixel(pixel, 255);
    }
 
  typedef  itk::ImageFileWriter< UnsignedCharImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("input.png");
  writer->SetInput(image);
  writer->Update();
}
