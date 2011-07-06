#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkImage.h"
#include "itkVector.h"
#include "itkDeformationFieldSource.h"

const     unsigned int   Dimension = 2;
typedef   unsigned char  PixelType;
typedef   itk::Image< PixelType, Dimension > ImageType;

void CreateImage(ImageType::Pointer image);
  
int main(int argc, char * argv[])
{
  
  typedef   float          VectorComponentType;

  typedef   itk::Vector< VectorComponentType, Dimension >    VectorType;
  typedef   itk::Image< VectorType,  Dimension >   DeformationFieldType;

  ImageType::Pointer fixedImage = ImageType::New();
  CreateImage(fixedImage);
  
  typedef itk::DeformationFieldSource<DeformationFieldType>  FilterType;
  FilterType::Pointer filter = FilterType::New();
  filter->SetOutputSpacing( fixedImage->GetSpacing() );
  filter->SetOutputOrigin(  fixedImage->GetOrigin() );
  filter->SetOutputRegion(  fixedImage->GetLargestPossibleRegion() );
  filter->SetOutputDirection( fixedImage->GetDirection() );

  //  Create source and target landmarks.
  typedef FilterType::LandmarkContainerPointer   LandmarkContainerPointer;
  typedef FilterType::LandmarkContainer          LandmarkContainerType;
  typedef FilterType::LandmarkPointType          LandmarkPointType;

  LandmarkContainerType::Pointer sourceLandmarks = LandmarkContainerType::New();
  LandmarkContainerType::Pointer targetLandmarks = LandmarkContainerType::New();

  LandmarkPointType sourcePoint;
  LandmarkPointType targetPoint;

  sourcePoint[0] = 40;
  sourcePoint[1] = 40;
  targetPoint[0] = 20;
  targetPoint[1] = 20;
  sourceLandmarks->InsertElement( 0, sourcePoint );
  targetLandmarks->InsertElement( 0, targetPoint );

  sourcePoint[0] = 40;
  sourcePoint[1] = 60;
  targetPoint[0] = 20;
  targetPoint[1] = 80;
  sourceLandmarks->InsertElement( 1, sourcePoint );
  targetLandmarks->InsertElement( 1, targetPoint );

  sourcePoint[0] = 60;
  sourcePoint[1] = 40;
  targetPoint[0] = 80;
  targetPoint[1] = 20;
  sourceLandmarks->InsertElement( 2, sourcePoint );
  targetLandmarks->InsertElement( 2, targetPoint );

  sourcePoint[0] = 60;
  sourcePoint[1] = 60;
  targetPoint[0] = 80;
  targetPoint[1] = 80;
  sourceLandmarks->InsertElement( 3, sourcePoint );
  targetLandmarks->InsertElement( 3, targetPoint );

  filter->SetSourceLandmarks( sourceLandmarks.GetPointer() );
  filter->SetTargetLandmarks( targetLandmarks.GetPointer() );
  filter->UpdateLargestPossibleRegion();

  // Write the deformation field
  typedef itk::ImageFileWriter<  DeformationFieldType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetInput (  filter->GetOutput() );
  writer->SetFileName( "deformation.mhd" );
  writer->Update();
  
  return EXIT_SUCCESS;
}

void CreateImage(ImageType::Pointer image)
{
  // Create a black image with a white square
  ImageType::IndexType start;
  start.Fill(0);
 
  ImageType::SizeType size;
  size.Fill(100);
 
  ImageType::RegionType region;
  region.SetSize(size);
  region.SetIndex(start);
 
  image->SetRegions(region);
  image->Allocate();
  image->FillBuffer(0);
 
  itk::ImageRegionIterator<ImageType> imageIterator(image,region);
 
  while(!imageIterator.IsAtEnd())
    {
    if(imageIterator.GetIndex()[0] > 40 && imageIterator.GetIndex()[0] < 60 &&
      imageIterator.GetIndex()[1] > 40 && imageIterator.GetIndex()[1] < 60)
      {
      imageIterator.Set(255);
      }
    ++imageIterator;
    }

  // Write the deformation field
  typedef itk::ImageFileWriter<  ImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetInput (  image );
  writer->SetFileName( "input.png" );
  writer->Update();
}
