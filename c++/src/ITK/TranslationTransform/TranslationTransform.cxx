#include "itkImage.h"
#include "itkTranslationTransform.h"
#include "itkImageFileReader.h"
#include "itkNormalizeImageFilter.h"
#include "itkResampleImageFilter.h"
#include "itkImageFileWriter.h"

#include "QuickView.h"

typedef itk::Image<unsigned char, 2>  ImageType;

void CreateImage(ImageType::Pointer image);

int main(int argc, char *argv[])
{
  ImageType::Pointer image = ImageType::New();
  CreateImage(image);

  typedef itk::TranslationTransform<double,2> TranslationTransformType;
  TranslationTransformType::Pointer transform =
    TranslationTransformType::New();
  TranslationTransformType::OutputVectorType translation;
  translation[0] = 10;
  translation[1] = 20;
  transform->Translate(translation);

  typedef itk::ResampleImageFilter<ImageType, ImageType> ResampleImageFilterType;
  ResampleImageFilterType::Pointer resampleFilter = ResampleImageFilterType::New();
  resampleFilter->SetTransform(transform.GetPointer());
  resampleFilter->SetInput(image);

  /*
  // These are the defaults
  double spacing[ 2 ];
  spacing[0] = 1.0;
  spacing[1] = 1.0;
  resampleFilter->SetOutputSpacing( spacing );

  double origin[ 2 ];
  origin[0] = 0.0;
  origin[1] = 0.0;
  resampleFilter->SetOutputOrigin( origin );
  */

  // Without this, the program crashes
  ImageType::SizeType   size = image->GetLargestPossibleRegion().GetSize();
  resampleFilter->SetSize( size );

  resampleFilter->Update();

  QuickView viewer;
  viewer.AddImage(image.GetPointer());
  viewer.AddImage(resampleFilter->GetOutput());
  viewer.Visualize();


  return EXIT_SUCCESS;
}

void CreateImage(ImageType::Pointer image)
{
  ImageType::IndexType start;
  start.Fill(0);

  ImageType::SizeType size;
  size.Fill(100);

  ImageType::RegionType region(start, size);
  image->SetRegions(region);
  image->Allocate();
  image->FillBuffer(0);

  // Make a square
  for(unsigned int r = 40; r < 60; r++)
    {
    for(unsigned int c = 40; c < 60; c++)
      {
      ImageType::IndexType pixelIndex;
      pixelIndex[0] = r;
      pixelIndex[1] = c;

      image->SetPixel(pixelIndex, 255);
      }
    }
}