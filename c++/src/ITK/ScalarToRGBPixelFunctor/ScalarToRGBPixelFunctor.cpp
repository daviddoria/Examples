#include <iostream>

#include "itkImageFileWriter.h"
#include "itkImageRegionIterator.h"
#include "itkScalarToRGBPixelFunctor.h"
#include "itkUnaryFunctorImageFilter.h"

// Run with:
// output.png

typedef itk::Image<unsigned long, 2>   LabeledImageType;

void SetPatch(LabeledImageType::Pointer image, itk::Index<2> index, unsigned long value);

int main( int argc, char *argv[] )
{
  // Verify arguments
  if (argc < 2 )
    {
    std::cerr << "outputImage" << std::endl;
    return 1;
    }

  // Parse arguments
  std::string outputFileName = argv[1];

  // Output arguments
  std::cout << "Running with:" << std::endl
            << "Output: " << outputFileName << std::endl;

  // Create an image with 3 regions

  LabeledImageType::Pointer image = LabeledImageType::New();
  itk::Size<2> size;
  size.Fill(100);

  itk::Index<2> index;
  index.Fill(0);

  itk::ImageRegion<2> region(index,size);
  image->SetRegions(region);
  image->Allocate();
  image->FillBuffer(0);

  // Set a region of the image to 1
  itk::Index<2> index1;
  index1.Fill(20);
  
  SetPatch(image, index1, 1);

  // Set a region of the image to 2
  itk::Index<2> index2;
  index2.Fill(40);

  SetPatch(image, index2, 2);

  // Set a region of the image to 3
  itk::Index<2> index3;
  index3.Fill(60);

  SetPatch(image, index3, 3);

  
  typedef itk::RGBPixel<unsigned char>   RGBPixelType;
  typedef itk::Image<RGBPixelType, 2>    RGBImageType;

  typedef itk::Functor::ScalarToRGBPixelFunctor<unsigned long> ColorMapFunctorType;
  typedef itk::UnaryFunctorImageFilter<LabeledImageType,
                                       RGBImageType, ColorMapFunctorType> ColorMapFilterType;
  ColorMapFilterType::Pointer colormapper = ColorMapFilterType::New();
  colormapper->SetInput(image);
  colormapper->Update();

  typedef itk::ImageFileWriter<RGBImageType> FileWriterType;
  FileWriterType::Pointer writer = FileWriterType::New();
  writer->SetFileName(outputFileName);
  writer->SetInput(colormapper->GetOutput());
  writer->Update();
  
  return EXIT_SUCCESS;
}

void SetPatch(LabeledImageType::Pointer image, itk::Index<2> index, unsigned long value)
{
  // Set a region of the image to 2
  itk::Size<2> size;
  size.Fill(20);

  itk::ImageRegion<2> region(index,size);
  itk::ImageRegionIterator<LabeledImageType> imageIterator(image,region);
  while(!imageIterator.IsAtEnd())
    {
    imageIterator.Set(value);

    ++imageIterator;
    }

}