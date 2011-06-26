#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkShapedFloodFilledImageFunctionConditionalIterator.h"
#include "itkBinaryThresholdImageFunction.h"
#include "itkImageFileWriter.h"

#include "QuickView.h"

typedef itk::Image< unsigned char, 2 >  ImageType;

void CreateImage(ImageType::Pointer image);

int main( int argc, char *argv[])
{
  ImageType::Pointer image = ImageType::New();
  CreateImage(image);

  typedef itk::BinaryThresholdImageFunction< ImageType, double > FunctionType;
  FunctionType::Pointer function = FunctionType::New();
  function->SetInputImage(image);
  function->ThresholdAbove(100); // we are looking to capture 255

  typedef itk::ShapedFloodFilledImageFunctionConditionalIterator< ImageType, FunctionType > IteratorType;

  itk::Index<2> seed;
  seed[0] = 5;
  seed[1] = 5;

  std::vector<itk::Index<2> > seeds;
  seeds.push_back(seed);

  IteratorType it (image, function, seeds);
  it.SetFullyConnected(true);
  it.GoToBegin();

  ImageType::Pointer outputImage = ImageType::New();
  outputImage->SetRegions(image->GetLargestPossibleRegion());
  outputImage->Allocate();
  outputImage->FillBuffer(0);

  unsigned int counter = 0;
  while ( !it.IsAtEnd() )
    {
    //it.Set(static_cast<float>(counter) * 25.);
    outputImage->SetPixel(it.GetIndex(), static_cast<float>(counter+1) * 25.);
    ++it;
    counter++;
    }

  typedef  itk::ImageFileWriter< ImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("output.png");
  writer->SetInput(image);
  writer->Update();


  ImageType::Pointer originalImage = ImageType::New();
  CreateImage(originalImage);

  QuickView viewer;
  viewer.AddImage(originalImage.GetPointer());
  //viewer.AddImage(image.GetPointer());
  viewer.AddImage(outputImage.GetPointer());
  viewer.Visualize();

  return EXIT_SUCCESS;
}

void CreateImage(ImageType::Pointer image)
{
  itk::Index<2> start;
  start.Fill(0);

  itk::Size<2> size;
  size.Fill(10);

  itk::ImageRegion<2> region(start,size);
  image->SetRegions(region);
  image->Allocate();
  image->FillBuffer(0);

  // Make a square
  for(unsigned int r = 3; r < 6; r++)
    {
    for(unsigned int c = 3; c < 6; c++)
      {
      itk::Index<2> pixelIndex;
      pixelIndex[0] = r;
      pixelIndex[1] = c;

      image->SetPixel(pixelIndex, 255);
      }
    }

  // Add this pixel to ensure only the region specified above is being visited
  itk::Index<2> zeroPixel;
  zeroPixel.Fill(0);
  image->SetPixel(zeroPixel, 255);
}