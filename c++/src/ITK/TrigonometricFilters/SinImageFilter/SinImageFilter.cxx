#include "itkImage.h"
#include "itkSinImageFilter.h"

typedef itk::Image<float, 2>  FloatImageType;

static void CreateImage(FloatImageType::Pointer image);

int main(int, char *[])
{
  FloatImageType::Pointer image = FloatImageType::New();
  CreateImage(image);

  // Compute the sine of each pixel
  typedef itk::SinImageFilter <FloatImageType, FloatImageType>
          SinImageFilterType;

  SinImageFilterType::Pointer sinImageFilter = SinImageFilterType::New ();
  sinImageFilter->SetInput(image);
  sinImageFilter->Update();

  return EXIT_SUCCESS;
}

void CreateImage(FloatImageType::Pointer image)
{
  FloatImageType::IndexType start;
  start.Fill(0);
  
  FloatImageType::SizeType size;
  size.Fill(10);

  FloatImageType::RegionType region(start,size);
  image->SetRegions(region);
  image->Allocate();
  image->FillBuffer(0);
  
}
