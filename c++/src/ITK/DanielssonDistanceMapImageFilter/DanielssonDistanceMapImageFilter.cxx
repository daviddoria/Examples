#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkBinomialBlurImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkDanielssonDistanceMapImageFilter.h"

typedef itk::Image<unsigned char, 2>  UnsignedCharImageType;
typedef itk::Image<float, 2>  FloatImageType;
 
static void CreateImage(UnsignedCharImageType::Pointer image);

int main(int argc, char * argv[])
{
  UnsignedCharImageType::Pointer image = UnsignedCharImageType::New();
  CreateImage(image);
  
  typedef  itk::DanielssonDistanceMapImageFilter< UnsignedCharImageType, FloatImageType  > DanielssonDistanceMapImageFilterType;
  DanielssonDistanceMapImageFilterType::Pointer danielssonDistanceMapImageFilter = DanielssonDistanceMapImageFilterType::New();
  danielssonDistanceMapImageFilter->SetInput(image);
  danielssonDistanceMapImageFilter->Update();
  
  typedef itk::RescaleIntensityImageFilter< FloatImageType, UnsignedCharImageType > RescaleFilterType;
  RescaleFilterType::Pointer rescaleFilter = RescaleFilterType::New();
  rescaleFilter->SetInput(danielssonDistanceMapImageFilter->GetOutput());
  rescaleFilter->SetOutputMinimum(0);
  rescaleFilter->SetOutputMaximum(255);
  rescaleFilter->Update();
  
  typedef  itk::ImageFileWriter< UnsignedCharImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("output.png");
  writer->SetInput(rescaleFilter->GetOutput());
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
 
  itk::ImageRegion<2> region(start, size);
  image->SetRegions(region);
  image->Allocate();
  image->FillBuffer(0);
 
  // Create a line of white pixels
  for(unsigned int i = 40; i < 60; ++i)
    {
    itk::Index<2> pixel;
    pixel.Fill(i);
    image->SetPixel(pixel, 255);
    }

  typedef  itk::ImageFileWriter< UnsignedCharImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("input.png");
  writer->SetInput(image);
  writer->Update();
}
