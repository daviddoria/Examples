#include "itkImage.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkVnlFFTRealToComplexConjugateImageFilter.h"
#include "itkComplexToRealImageFilter.h"
#include "itkComplexToImaginaryImageFilter.h"
#include "itkComplexToModulusImageFilter.h"
#include "itkImageFileReader.h"
#include "itkCastImageFilter.h"
#include "itkPasteImageFilter.h"
#include "itkContourMeanDistanceImageFilter.h"

#include <itksys/SystemTools.hxx>
#include "vnl/vnl_sample.h"
#include <math.h>

#include <itkImageToVTKImageFilter.h>

#include "QuickView.h"

typedef itk::Image<unsigned char, 2> ImageType;

void CreateImage1(ImageType::Pointer);
void CreateImage2(ImageType::Pointer);

int main(int argc, char*argv[])
{
  ImageType::Pointer image1 = ImageType::New();
  CreateImage1(image1);

  ImageType::Pointer image2 = ImageType::New();
  CreateImage2(image2);

  typedef itk::ContourMeanDistanceImageFilter <ImageType, ImageType >
    ContourMeanDistanceImageFilterType;

  ContourMeanDistanceImageFilterType::Pointer contourMeanDistanceImageFilter =
    ContourMeanDistanceImageFilterType::New();
  contourMeanDistanceImageFilter->SetInput1(image1);
  contourMeanDistanceImageFilter->SetInput2(image2);
  contourMeanDistanceImageFilter->Update();

  std::cout << "Mean distance: " << contourMeanDistanceImageFilter->GetMeanDistance() << std::endl;

  QuickView viewer;
  viewer.AddImage(image1.GetPointer());
  viewer.AddImage(image2.GetPointer());
  viewer.Visualize();

  return EXIT_SUCCESS;
}

void CreateImage1(ImageType::Pointer image)
{
  // Create an image bigger than the input image and that has dimensions which are powers of two
  itk::Index<2> start;
  start.Fill(0);

  itk::Size<2> size;
  size.Fill(20);

  itk::ImageRegion<2> region(start, size);

  image->SetRegions(region);
  image->Allocate();

  for(unsigned int i = 0; i < 20; i++)
    {
    for(unsigned int j = 0; j < 20; j++)
      {
      if(i == j) // y = x
        {
        itk::Index<2> pixel;
        pixel[0] = i;
        pixel[1] = j;
        image->SetPixel(pixel, 255);
        }
      }
    }
}

void CreateImage2(ImageType::Pointer image)
{
  // Create an image bigger than the input image and that has dimensions which are powers of two
  itk::Index<2> start;
  start.Fill(0);

  itk::Size<2> size;
  size.Fill(20);

  itk::ImageRegion<2> region(start, size);

  image->SetRegions(region);
  image->Allocate();

  for(unsigned int i = 0; i < 20; i++)
    {
    for(unsigned int j = 0; j < 20; j++)
      {
      if(i == 10)
        {
        itk::Index<2> pixel;
        pixel[0] = i;
        pixel[1] = j;
        image->SetPixel(pixel, 255);
        }
      }
    }
}
