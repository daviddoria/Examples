#include "itkBinomialBlurImageFilter.h"
#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkScalarConnectedComponentImageFilter.h"

typedef itk::Image< unsigned char, 2 >  ImageType;

static void CreateImage(ImageType::Pointer image);

int main( int argc, char *argv[])
{

  ImageType::Pointer image = ImageType::New();
  CreateImage(image);
  
  typedef itk::ScalarConnectedComponentImageFilter<ImageType, ImageType> ScalarConnectedComponentImageFilterType;
  ScalarConnectedComponentImageFilterType::Pointer scalarConnectedComponentFilter = ScalarConnectedComponentImageFilterType::New();
  scalarConnectedComponentFilter->SetInput(image);
  scalarConnectedComponentFilter->Update();

  typedef  itk::ImageFileWriter< ImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("output.png");
  writer->SetInput(scalarConnectedComponentFilter->GetOutput());
  writer->Update();
  
  return EXIT_SUCCESS;
}

void CreateImage(ImageType::Pointer image)
{
  // Create an image with 2 connected components
  ImageType::IndexType start;
  start.Fill(0);

  ImageType::SizeType size;
  size.Fill(100);

  ImageType::RegionType region(start,size);

  image->SetRegions(region);
  image->Allocate();

  // Make a square
  for(unsigned int r = 20; r < 80; r++)
    {
    for(unsigned int c = 30; c < 100; c++)
      {
      ImageType::IndexType pixelIndex;
      pixelIndex[0] = r;
      pixelIndex[1] = c;

      image->SetPixel(pixelIndex, 255);
      }
    }
  
  typedef itk::BinomialBlurImageFilter<ImageType, ImageType >  BinomialBlurImageFilterType;
  BinomialBlurImageFilterType::Pointer binomialBlurImageFilter = BinomialBlurImageFilterType::New();
  binomialBlurImageFilter->SetInput( image );
  binomialBlurImageFilter->SetRepetitions( 4 );
  binomialBlurImageFilter->Update();
  
  image->Graft(binomialBlurImageFilter->GetOutput());

}
