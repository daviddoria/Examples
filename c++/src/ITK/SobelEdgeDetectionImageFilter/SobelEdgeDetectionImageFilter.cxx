#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkSobelEdgeDetectionImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"

typedef itk::Image<unsigned char, 2>  UnsignedCharImageType;
typedef itk::Image<float, 2>  FloatImageType;

static void CreateImage(UnsignedCharImageType::Pointer);

int main(int argc, char *argv[])
{
  UnsignedCharImageType::Pointer image = UnsignedCharImageType::New();
  CreateImage(image);

  typedef itk::SobelEdgeDetectionImageFilter <UnsignedCharImageType, FloatImageType>
          SobelEdgeDetectionImageFilterType;
  SobelEdgeDetectionImageFilterType::Pointer sobelFilter
          = SobelEdgeDetectionImageFilterType::New();
  sobelFilter->SetInput(image);
  sobelFilter->Update();

  typedef itk::RescaleIntensityImageFilter< FloatImageType, UnsignedCharImageType > RescaleFilterType;
  RescaleFilterType::Pointer rescaleFilter = RescaleFilterType::New();
  rescaleFilter->SetInput(sobelFilter->GetOutput());
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
  itk::Index<2> start;
  start.Fill(0);

  itk::Size<2> size;
  size.Fill(100);

  itk::ImageRegion<2> region(start, size);
  image->SetRegions(region);
  image->Allocate();

  // Make a square
  for(unsigned int r = 20; r < 80; r++)
    {
    for(unsigned int c = 20; c < 80; c++)
      {
      itk::Index<2> pixelIndex;
      pixelIndex[0] = r;
      pixelIndex[1] = c;

      image->SetPixel(pixelIndex, 15);
      }
    }
}
