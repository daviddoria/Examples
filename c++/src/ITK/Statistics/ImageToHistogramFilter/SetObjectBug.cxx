#include "itkImageToHistogramFilter.h"
#include "itkImage.h"
#include "itkRGBPixel.h"
#include "itkImageRegionIteratorWithIndex.h"

typedef itk::RGBPixel<unsigned char> RGBPixelType;
typedef itk::Image< RGBPixelType, 2> RGBImageType;

void CreateImage(RGBImageType::Pointer image);

int main(int, char *[])
{
  const unsigned int MeasurementVectorSize = 3; // RGB

  RGBImageType::Pointer image = RGBImageType::New();
  CreateImage(image);

  typedef itk::Statistics::ImageToHistogramFilter< RGBImageType >         HistogramFilterType;
  typedef HistogramFilterType::HistogramMeasurementVectorType             HistogramMeasurementVectorType;
  typedef HistogramFilterType::HistogramSizeType                          HistogramSizeType;
  typedef HistogramFilterType::HistogramType                              HistogramType;

  HistogramFilterType::Pointer filter = HistogramFilterType::New();
  filter->SetInput(image);

  typedef itk::SimpleDataObjectDecorator<HistogramMeasurementVectorType> ObjectType;

  HistogramMeasurementVectorType min;
  min.Fill(0);

  ObjectType::Pointer minObject = ObjectType::New();
  minObject->Set(min);
  filter->SetHistogramBinMinimumInput(minObject);

  HistogramMeasurementVectorType max;
  max.Fill(255);
  ObjectType::Pointer maxObject = ObjectType::New();
  maxObject->Set(max);
  filter->SetHistogramBinMaximumInput(maxObject);

  HistogramSizeType histogramSize( MeasurementVectorSize );

  histogramSize[0] = 2;  // number of bins for the Red   channel
  histogramSize[1] = 2;  // number of bins for the Green channel
  histogramSize[2] = 2;  // number of bins for the Blue  channel

  filter->SetHistogramSize(histogramSize);
  filter->SetMarginalScale(10); // Required (could this be set in the filter?)
  filter->Update();


  return EXIT_SUCCESS;
}

void CreateImage(RGBImageType::Pointer image)
{
  // Create a black image with a red square and a green square.
  // This should produce a histogram with very strong spikes.
  RGBImageType::SizeType   size;
  size[0] = 3;
  size[1] = 3;

  RGBImageType::IndexType  start;
  start[0] = 0;
  start[1] = 0;

  RGBImageType::RegionType region;
  region.SetIndex(start);
  region.SetSize(size);

  image->SetRegions(region);
  image->Allocate();

  itk::ImageRegionIteratorWithIndex< RGBImageType > iterator( image, image->GetLargestPossibleRegion() );
  iterator.GoToBegin();

  RGBPixelType blackPixel;
  blackPixel.SetRed(0);
  blackPixel.SetGreen(0);
  blackPixel.SetBlue(0);

  RGBPixelType redPixel;
  redPixel.SetRed(255);
  redPixel.SetGreen(0);
  redPixel.SetBlue(0);

  RGBPixelType greenPixel;
  greenPixel.SetRed(0);
  greenPixel.SetGreen(255);
  greenPixel.SetBlue(0);

  RGBPixelType bluePixel;
  bluePixel.SetRed(0);
  bluePixel.SetGreen(0);
  bluePixel.SetBlue(255);

  itk::ImageRegionIterator<RGBImageType> imageIterator(image,region);

  while(!imageIterator.IsAtEnd())
    {
    imageIterator.Set(blackPixel);
    ++imageIterator;
    }

  RGBImageType::IndexType index;
  index[0] = 0; index[1] = 0;
  image->SetPixel(index, redPixel);


  index[0] = 1; index[1] = 0;
  image->SetPixel(index, greenPixel);

  index[0] = 1; index[1] = 1;
  image->SetPixel(index, bluePixel);

}