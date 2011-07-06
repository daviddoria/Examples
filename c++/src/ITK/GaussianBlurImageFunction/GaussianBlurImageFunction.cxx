#include "itkGaussianBlurImageFunction.h"
#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkImageRegionIterator.h"
#include "itkImageFunctionImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"

typedef itk::Image< unsigned char, 2 > UnsignedCharImageType;
typedef itk::Image< float, 2 > FloatImageType;

void CreateImage(UnsignedCharImageType::Pointer image);

int main( int argc, char * argv[] )
{
  UnsignedCharImageType::Pointer image = UnsignedCharImageType::New();
  CreateImage(image);
  
  typedef itk::GaussianBlurImageFunction< UnsignedCharImageType > GaussianBlurImageFunctionType;
  GaussianBlurImageFunctionType::Pointer gaussianFunction = GaussianBlurImageFunctionType::New();
  gaussianFunction->SetInputImage( image ); // Do we need to do this since we are going to give the input to the ImageFunctionImageFilter?
  GaussianBlurImageFunctionType::ErrorArrayType setError;
  setError.Fill( 0.01 );
  gaussianFunction->SetMaximumError( setError );
  gaussianFunction->SetSigma( 1.0);
  gaussianFunction->SetMaximumKernelWidth( 3 );
    
  typedef itk::ImageFunctionImageFilter<UnsignedCharImageType,FloatImageType,
                                  GaussianBlurImageFunctionType> ImageFunctionImageFilterType;
  ImageFunctionImageFilterType::Pointer imageFunctionImageFilter = ImageFunctionImageFilterType::New();
  imageFunctionImageFilter->SetInput(image);
  imageFunctionImageFilter->SetFunction(gaussianFunction);
  //imageFunctionImageFilter->SetNumberOfThreads(1);
  imageFunctionImageFilter->Update();

  typedef itk::RescaleIntensityImageFilter< FloatImageType, UnsignedCharImageType > RescaleFilterType;
  RescaleFilterType::Pointer rescaleFilter = RescaleFilterType::New();
  rescaleFilter->SetInput(imageFunctionImageFilter->GetOutput());
  rescaleFilter->SetOutputMinimum(0);
  rescaleFilter->SetOutputMaximum(255);
  rescaleFilter->Update();

  typedef itk::ImageFileWriter < UnsignedCharImageType > WriterType;
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
 
  itk::ImageRegion<2> region(start,size);
 
  image->SetRegions(region);
  image->Allocate();
  image->FillBuffer(0);
 
  itk::ImageRegionIterator<UnsignedCharImageType> imageIterator(image,region);
 
  while(!imageIterator.IsAtEnd())
    {
    if(imageIterator.GetIndex()[0] >= 50 && imageIterator.GetIndex()[1] >= 50 &&
       imageIterator.GetIndex()[0] <= 70 && imageIterator.GetIndex()[1] <= 70)
      {
      imageIterator.Set(255);
      }
 
    ++imageIterator;
    }
 
  typedef itk::ImageFileWriter < UnsignedCharImageType > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("input.png");
  writer->SetInput(image);
  writer->Update();
}
