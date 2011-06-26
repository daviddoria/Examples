#include "itkBSplineScatteredDataPointSetToImageFilter.h"
#include "itkPointSet.h"
#include "itkImage.h"
#include "itkVectorImage.h"
#include "itkImageFileWriter.h"

int main (void)
{
  const unsigned int ParametricDimension = 1;
  const unsigned int DataDimension = 2;

  typedef itk::Vector< float, DataDimension > DataType;

  typedef itk::PointSet< DataType, ParametricDimension >   PointSetType;

  PointSetType::Pointer pointSet = PointSetType::New();

  PointSetType::PointType param0, param1, param2;

  param0[0] = 0.0;
  DataType p0;
  p0[0] =  1.0; p0[1]= 1.0;

  pointSet->SetPoint(0, param0);
  pointSet->SetPointData( 0, p0 );

  param1[0] = 1.0;
  DataType p1;
  p1[0] =  100.0; p1[1]= 50.0;
  pointSet->SetPoint(1, param1);
  pointSet->SetPointData( 1, p1 );

  param2[0] = 2.0;
  DataType p2;
  p2[0] =  100.0; p2[1]= 1.0;
  pointSet->SetPoint(2, param2);
  pointSet->SetPointData( 2, p2 );

  typedef itk::Image<DataType, ParametricDimension> ImageType;
  typedef itk::BSplineScatteredDataPointSetToImageFilter < PointSetType, ImageType > SplineFilterType;
  SplineFilterType::Pointer splineFilter = SplineFilterType::New();

  SplineFilterType::ArrayType ncontrol;
  ncontrol[0]=20;
  SplineFilterType::ArrayType closedim;
  closedim[0]= 0;
  int splineorder=2; // complexity of the spline

  ImageType::PointType parametricDomainOrigin;
  parametricDomainOrigin[0] = 0.0;

  ImageType::SpacingType parametricDomainSpacing;
  parametricDomainSpacing[0] = 0.01;  // this determines the sampling of the continuous B-spline object.

  ImageType::SizeType parametricDomainSize;
  parametricDomainSize[0] = 2.0 / parametricDomainSpacing[0] + 1;

  splineFilter->SetGenerateOutputImage( true );   // the only reason to turn this off is if one only wants to use the control point lattice for further processing
  splineFilter->SetInput ( pointSet );
  splineFilter->SetSplineOrder ( splineorder );
  splineFilter->SetNumberOfControlPoints ( ncontrol );
  splineFilter->SetNumberOfLevels( 1 );
  splineFilter->SetCloseDimension ( closedim );
  splineFilter->SetSize( parametricDomainSize );
  splineFilter->SetSpacing( parametricDomainSpacing );
  splineFilter->SetOrigin( parametricDomainOrigin );
  splineFilter->Update();

  std::cout << splineFilter->GetOutput()->GetLargestPossibleRegion().GetSize();

  // The output will consist of a 1-D image where each voxel contains the
  // (x,y,z) locations of the points
  typedef itk::Image<unsigned char, 2> OutputImageType;
  OutputImageType::Pointer outputImage = OutputImageType::New();
  OutputImageType::SizeType size;
  size.Fill(200);

  OutputImageType::IndexType start;
  start.Fill(0);

  OutputImageType::RegionType region(start, size);
  outputImage->SetRegions(region);

  outputImage->FillBuffer(0);

  for(unsigned int i = 0; i < splineFilter->GetOutput()->GetLargestPossibleRegion().GetSize()[0]; ++i)
    {
    ImageType::IndexType splineIndex;
    splineIndex[0] = i;

    DataType outputPixel = splineFilter->GetOutput()->GetPixel(splineIndex);

    OutputImageType::IndexType index;
    index[0] = outputPixel[0];
    index[1] = outputPixel[1];

    outputImage->SetPixel(index, 255);
    }

  typedef  itk::ImageFileWriter< OutputImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("spline.png");
  writer->SetInput(outputImage);
  writer->Update();

  return EXIT_SUCCESS;
 };
