#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkOtsuThresholdImageFilter.h"
#include "itkImageRegionIterator.h"

typedef itk::Image<unsigned char, 2>  ImageType;

void CreateImage(ImageType::Pointer image);

int main(int, char *[])
{
  typedef itk::Image<float, 2> ImageType;
  ImageType::Pointer sumimage = ImageType::New();
  ImageType::Pointer sumimage1 = ImageType::New();
  ImageType::SizeType size;

  //THIS WORKS
  size[0]  = src_img_in.GetDimensionSize( 0 );
  size[1]  = src_img_in.GetDimensionSize( 1 );
  //
  ImageType::IndexType start;
  start.Fill(0);

  ImageType::RegionType region;
  region.SetIndex( start );
  region.SetSize( size );

  sumimage->SetLargestPossibleRegion( region );
  sumimage->SetBufferedRegion( region );
  sumimage->SetRequestedRegion( region );
  sumimage->Allocate();


  sumimage1->SetLargestPossibleRegion( region );
  sumimage1->SetBufferedRegion( region );
  sumimage1->SetRequestedRegion( region );
  sumimage1->Allocate();
  typedef itk::Statistics::ScalarImageToHistogramGenerator<ImageType >
  ScalarImageToHistogramGeneratorType;
  typedef
  itk::OtsuMultipleThresholdsCalculator<ScalarImageToHistogramGeneratorType::HistogramType
  > CalculatorType;
  typedef itk::BinaryThresholdImageFilter< ImageType, ImageType >
  FilterType;
  typedef itk::AddImageFilter< ImageType, ImageType ,ImageType>
  AddFilterType;

  ScalarImageToHistogramGeneratorType::Pointer
  scalarImageToHistogramGenerator =
  ScalarImageToHistogramGeneratorType::New();
  CalculatorType::Pointer calculator = CalculatorType::New();
  FilterType::Pointer filter = FilterType::New();
  AddFilterType::Pointer addFilter = AddFilterType::New();

  scalarImageToHistogramGenerator->SetNumberOfBins(n_bins);
  calculator->SetNumberOfThresholds(n_trsh);
  //THIS WORKS
  if(flag) scalarImageToHistogramGenerator->SetInput(dmtoitk2(src_img_in));
  else scalarImageToHistogramGenerator->SetInput(dmtoitk2(src_img));
  //

  calculator->SetInputHistogram(scalarImageToHistogramGenerator->GetOutput());
  filter->SetInput(dmtoitk2(src_img_in) );

  scalarImageToHistogramGenerator->Compute();
  calculator->Update();

  const CalculatorType::OutputType &thresholdVector =
  calculator->GetOutput();
  CalculatorType::OutputType::const_iterator itNum =
  thresholdVector.begin();

  //THIS WORKS
  DM::ImageCalculateMinMax( src_img_in,1, 1, &min, &max );
  //
  const float outsideValue = 0;


  float lowerThreshold = min;
  float upperThreshold;

  filter->SetOutsideValue( outsideValue );

  for(; itNum < thresholdVector.end(); itNum++)
  {

  const float insideValue = count;
  filter->SetInsideValue( insideValue );

  upperThreshold = static_cast<float>(*itNum);

  filter->SetLowerThreshold( lowerThreshold );
  filter->SetUpperThreshold( upperThreshold );
  filter->Update();

  lowerThreshold = upperThreshold;

  addFilter->SetInput1( filter->GetOutput() );
  addFilter->SetInput2(  sumimage );
  //sumimage = addFilter->GetOutput( );
  addFilter->Update();
  sumimage1 = addFilter->GetOutput() ;
  sumimage = sumimage1;

  count++;
  }
  const float insideValue = count;


  filter->SetInsideValue( count  );
  filter->SetLowerThreshold( lowerThreshold );
  filter->SetUpperThreshold( max );
  filter->Update();


  addFilter->SetInput1( filter->GetOutput() );
  addFilter->SetInput2(  sumimage );
  //sumimage = addFilter->GetOutput( );
  addFilter->Update();
  sumimage1 = addFilter->GetOutput() ;
  //THIS WORKS
  l_deriv_img_out = itktodm2( sumimage1);
  l_deriv_img_out.SetName(( std::string("Segmentated ") + name ).c_str());

  return EXIT_SUCCESS;
}

void CreateImage(ImageType::Pointer image)
{
  // Create an image
  ImageType::IndexType start;
  start.Fill(0);

  ImageType::SizeType size;
  size.Fill(100);

  ImageType::RegionType region;
  region.SetSize(size);
  region.SetIndex(start);

  image->SetRegions(region);
  image->Allocate();

  // Make the whole image white
  itk::ImageRegionIterator<ImageType> iterator(image,image->GetLargestPossibleRegion());

  /*
   //Create a square
  while(!iterator.IsAtEnd())
    {
    iterator.Set(255);
    ++iterator;
    }
  */
}
