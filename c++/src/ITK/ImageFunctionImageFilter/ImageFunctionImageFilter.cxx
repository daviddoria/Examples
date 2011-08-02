#include "itkGaussianBlurImageFunction.h"
#include "itkImageFunctionImageFilter.h"
#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkIndex.h"
//#include "itkRescaleIntensityImageFilter.h"
#include "itkImageRegionIterator.h"

#include <iostream>

namespace // anonymous
{
  typedef itk::Image< unsigned char, 2 > UnsignedCharImageType;
  typedef itk::Image< float, 2 > FloatImageType;

  static void CreateImage(UnsignedCharImageType::Pointer image);
}
//-------------------------
//
//   This file tests the interface to the ImageFunctionImageFilter class
//
//-------------------------
int main(int, char* [] )
{
  UnsignedCharImageType::Pointer image = UnsignedCharImageType::New();
  CreateImage(image);

  typedef itk::GaussianBlurImageFunction< UnsignedCharImageType > GaussianBlurImageFunctionType;
  GaussianBlurImageFunctionType::Pointer gaussianFunction = GaussianBlurImageFunctionType::New();
  GaussianBlurImageFunctionType::ErrorArrayType setError;
  setError.Fill( 0.01 );
  gaussianFunction->SetMaximumError( setError );
  gaussianFunction->SetSigma( 1.0);
  gaussianFunction->SetMaximumKernelWidth( 3 );

  typedef itk::ImageFunctionImageFilter<UnsignedCharImageType,FloatImageType,GaussianBlurImageFunctionType> ImageFunctionImageFilterType;
  ImageFunctionImageFilterType::Pointer imageFunctionImageFilter = ImageFunctionImageFilterType::New();
  imageFunctionImageFilter->SetInput(image);
  imageFunctionImageFilter->SetFunction(gaussianFunction);
  imageFunctionImageFilter->Update();

  // Write the output file
  typedef  itk::ImageFileWriter< FloatImageType > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("ImageFunctionImageFilter.mha");
  writer->SetInput(imageFunctionImageFilter->GetOutput());
  writer->Update();

}

namespace // anonymous
{
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

}
} // end namespace
