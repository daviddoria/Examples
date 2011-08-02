#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkFloodFilledImageFunctionConditionalIterator.h"
#include "itkBinaryThresholdImageFunction.h"
#include "itkImageFileWriter.h"

typedef itk::Image< unsigned char, 2 >  ImageType;

static void CreateImage(ImageType::Pointer image);

int main( int argc, char *argv[])
{
  ImageType::Pointer image = ImageType::New();
  CreateImage(image);

  typedef itk::BinaryThresholdImageFunction< ImageType, double > FunctionType;
  FunctionType::Pointer function = FunctionType::New();
  function->SetInputImage(image);
  function->ThresholdAbove(100); // we are looking to capture 255

  typedef itk::FloodFilledImageFunctionConditionalIterator< ImageType, FunctionType > IteratorType;

  itk::Index<2> seed;
  seed[0] = 25;
  seed[1] = 25;

  std::vector<itk::Index<2> > seeds;
  seeds.push_back(seed);

  IteratorType it (image, function, seeds);
  it.GoToBegin();

  while ( !it.IsAtEnd() )
    {
    std::cout << it.GetIndex() << std::endl;
    ++it;
    }

  return EXIT_SUCCESS;
}

void CreateImage(ImageType::Pointer image)
{
  itk::Index<2> start;
  start.Fill(0);

  itk::Size<2> size;
  size.Fill(100);

  itk::ImageRegion<2> region(start,size);
  image->SetRegions(region);
  image->Allocate();
  image->FillBuffer(0);


  // Make a line
  for(unsigned int i = 20; i < 50; ++i)
    {
    itk::Index<2> pixelIndex;
    pixelIndex.Fill(i);

    image->SetPixel(pixelIndex, 255);
    }
  
  
  // Make a square
//   for(unsigned int r = 20; r < 50; r++)
//     {
//     for(unsigned int c = 20; c < 50; c++)
//       {
//       itk::Index<2> pixelIndex;
//       pixelIndex[0] = r;
//       pixelIndex[1] = c;
//  
//       image->SetPixel(pixelIndex, 255);
//       }
//     }
}
