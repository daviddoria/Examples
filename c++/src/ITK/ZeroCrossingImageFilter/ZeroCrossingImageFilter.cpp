#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkZeroCrossingImageFilter.h"

typedef itk::Image<float, 2>  FloatImageType;

static void CreateImage(FloatImageType::Pointer image);

int main(int argc, char *argv[])
{
  typedef itk::Image<unsigned char, 2>  UnsignedCharImageType;

  FloatImageType::Pointer image = FloatImageType::New();
  CreateImage(image);

  typedef  itk::ZeroCrossingImageFilter< FloatImageType, UnsignedCharImageType  > ZeroCrossingImageFilterType;
  ZeroCrossingImageFilterType::Pointer zeroCrossingImageFilter = ZeroCrossingImageFilterType::New();
  zeroCrossingImageFilter->SetInput(image);
  zeroCrossingImageFilter->SetBackgroundValue(0);
  zeroCrossingImageFilter->SetForegroundValue(255);
  zeroCrossingImageFilter->Update();
    
  typedef  itk::ImageFileWriter< UnsignedCharImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("output.png");
  writer->SetInput(zeroCrossingImageFilter->GetOutput());
  writer->Update();

  return 0;
}

void CreateImage(FloatImageType::Pointer image)
{
  itk::Index<2> start;
  start.Fill(0);

  itk::Size<2> size;
  size.Fill(100);

  itk::ImageRegion<2> region(start,size);
  
  image->SetRegions(region);
  image->Allocate();
  image->FillBuffer(-1);
  
  // Make half of the image negative
  for(unsigned int i = 0; i < 100; ++i)
    {
    for(unsigned int j = 0; j < 50; ++j)
      {
      itk::Index<2> index;
      index[0] = i;
      index[1] = j;
      image->SetPixel(index, 1);
      }
    }
}
