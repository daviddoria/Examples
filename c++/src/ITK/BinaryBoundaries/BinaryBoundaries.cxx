#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkInvertIntensityImageFilter.h"
#include "itkBinaryContourImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"

#include "QuickView.h"

typedef itk::Image<unsigned char, 2>  ImageType;

void CreateImage(ImageType::Pointer image);

int main(int, char *[])
{
  ImageType::Pointer image = ImageType::New();
  CreateImage(image);

  typedef itk::BinaryContourImageFilter <ImageType, ImageType >
          binaryContourImageFilterType;

  // Outer boundary
  binaryContourImageFilterType::Pointer binaryContourFilter
          = binaryContourImageFilterType::New ();
  binaryContourFilter->SetInput(image);
  binaryContourFilter->SetForegroundValue(0);
  binaryContourFilter->SetBackgroundValue(255);
  binaryContourFilter->Update();

  // Invert the result
  typedef itk::InvertIntensityImageFilter <ImageType>
          InvertIntensityImageFilterType;

  InvertIntensityImageFilterType::Pointer invertIntensityFilter
          = InvertIntensityImageFilterType::New();
  invertIntensityFilter->SetInput(binaryContourFilter->GetOutput());
  invertIntensityFilter->Update();

  ImageType::Pointer outerBoundary = ImageType::New();
  outerBoundary->Graft(invertIntensityFilter->GetOutput());

  // Inner boundary
  binaryContourFilter->SetForegroundValue(255);
  binaryContourFilter->SetBackgroundValue(0);
  binaryContourFilter->Update();

  QuickView viewer;
  viewer.AddImage(image.GetPointer());
  viewer.AddImage(outerBoundary.GetPointer());
  viewer.AddImage(binaryContourFilter->GetOutput());
  viewer.Visualize();

  return EXIT_SUCCESS;
}

void CreateImage(ImageType::Pointer image)
{
  ImageType::IndexType start;
  start.Fill(0);

  ImageType::SizeType size;
  size.Fill(20);

  ImageType::RegionType region(start, size);

  image->SetRegions(region);
  image->Allocate();

  // Make a square
  for(unsigned int r = 5; r < 10; r++)
    {
    for(unsigned int c = 5; c < 10; c++)
      {
      ImageType::IndexType pixelIndex;
      pixelIndex[0] = r;
      pixelIndex[1] = c;

      image->SetPixel(pixelIndex, 255);
      }
    }

}