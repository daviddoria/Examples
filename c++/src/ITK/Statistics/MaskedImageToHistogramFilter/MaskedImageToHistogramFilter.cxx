#include "itkMaskedImageToHistogramFilter.h"
#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkRGBPixel.h"
#include "itkImageRegionIteratorWithIndex.h"
#include "itkImageRegionIterator.h"
#include "itkRescaleIntensityImageFilter.h"

typedef itk::RGBPixel<unsigned char> RGBPixelType;
typedef itk::Image< RGBPixelType, 2> RGBImageType;

typedef itk::Image< unsigned char, 2> UnsignedCharImageType;

static void CreateImage(RGBImageType::Pointer image);
static void CreateHalfMask(itk::ImageRegion<2>, UnsignedCharImageType::Pointer mask);

int main(int, char *[])
{
  const unsigned int MeasurementVectorSize = 3; // RGB

  RGBImageType::Pointer image = RGBImageType::New();
  CreateImage(image);

  UnsignedCharImageType::Pointer mask = UnsignedCharImageType::New();
  CreateHalfMask(image->GetLargestPossibleRegion(), mask);
  
  typedef itk::Statistics::MaskedImageToHistogramFilter< RGBImageType, UnsignedCharImageType >   HistogramFilterType;
  typedef HistogramFilterType::HistogramMeasurementVectorType             HistogramMeasurementVectorType;
  typedef HistogramFilterType::HistogramSizeType                          HistogramSizeType;
  typedef HistogramFilterType::HistogramType                              HistogramType;

  HistogramFilterType::Pointer histogramFilter = HistogramFilterType::New();
  histogramFilter->SetInput(image);
  histogramFilter->SetMaskImage(mask);
  histogramFilter->SetAutoMinimumMaximum(true);

  HistogramSizeType histogramSize( MeasurementVectorSize );

  histogramSize[0] = 4;  // number of bins for the Red   channel
  histogramSize[1] = 4;  // number of bins for the Green channel
  histogramSize[2] = 4;  // number of bins for the Blue  channel

  histogramFilter->SetHistogramSize(histogramSize);
  histogramFilter->SetMarginalScale(10); // Required (could this be set in the filter?)
  histogramFilter->Update();

  const HistogramType * histogram = histogramFilter->GetOutput();

  HistogramType::ConstIterator histogramIterator = histogram->Begin();

  //std::string filename = "/home/doriad/histogram.txt";
  //std::ofstream fout(filename.c_str());

  while( histogramIterator  != histogram->End() )
    {
    //std::cout << "Index = " << histogramIterator.GetMeasurementVector() << "Frequency = " << histogramIterator.GetFrequency() << std::endl;
    //std::cout << "Index = " << histogramIterator.GetIndex() << "Frequency = " << histogramIterator.GetFrequency() << std::endl;
    //fout << "Index = " << histogram->GetIndex(histogramItr.GetMeasurementVector()) << "Frequency = " << histogramItr.GetFrequency() << std::endl;
    ++histogramIterator ;
    }
  //fout.close();

  HistogramType::MeasurementVectorType mv(3);
  mv[0] = 255;
  mv[1] = 0;
  mv[2] = 0;
  std::cout << "Frequency = " << histogram->GetFrequency(histogram->GetIndex(mv)) << std::endl;
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

  RGBPixelType redPixel;
  redPixel.SetRed(255);
  redPixel.SetGreen(0);
  redPixel.SetBlue(0);

  RGBPixelType blackPixel;
  blackPixel.SetRed(0);
  blackPixel.SetGreen(0);
  blackPixel.SetBlue(0);

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
  image->SetPixel(index, redPixel);
}

void CreateHalfMask(itk::ImageRegion<2> region, UnsignedCharImageType::Pointer mask)
{
  mask->SetRegions(region);
  mask->Allocate();
  mask->FillBuffer(0);
 
  itk::Size<2> regionSize = region.GetSize();
 
  itk::ImageRegionIterator<UnsignedCharImageType> imageIterator(mask,region);
 
  // Make the left half of the mask white and the right half black
  while(!imageIterator.IsAtEnd())
  {
    if(imageIterator.GetIndex()[0] > regionSize[0] / 2)
        {
        imageIterator.Set(0);
        }
      else
        {
        imageIterator.Set(1);
        }
 
    ++imageIterator;
  }
 
  typedef itk::RescaleIntensityImageFilter< UnsignedCharImageType, UnsignedCharImageType > RescaleFilterType;
  RescaleFilterType::Pointer rescaleFilter = RescaleFilterType::New();
  rescaleFilter->SetInput(mask);
  rescaleFilter->Update();
 
  typedef  itk::ImageFileWriter< UnsignedCharImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("mask.png");
  writer->SetInput(rescaleFilter->GetOutput());
  writer->Update();
 
}

