#include "itkImage.h"
#include "itkSphereSpatialFunction.h"
#include "itkFloodFilledSpatialFunctionConditionalIterator.h"

#include "QuickView.h"

typedef itk::Image< unsigned char, 2 >  TImageType;

void CreateImage(TImageType::Pointer image);

int main( int argc, char *argv[])
{
  TImageType::Pointer image = TImageType::New();
  CreateImage(image);

  typedef itk::SphereSpatialFunction<2> TFunctionType;

  // Create and initialize a new sphere function
  TFunctionType::Pointer spatialFunc = TFunctionType::New();
  spatialFunc->SetRadius(5);

  TFunctionType::InputType center;
  center[0]=10;
  center[1]=10;

  spatialFunc->SetCenter(center);

  //---------Create and initialize a spatial function iterator-----------
  itk::Index<2> seedPos;
  seedPos[0] = 5;
  seedPos[1] = 5;

  typedef itk::FloodFilledSpatialFunctionConditionalIterator <TImageType, TFunctionType> TItType;
  TItType iterator = TItType(image, spatialFunc, seedPos);

  for( ; !( iterator.IsAtEnd() ); ++iterator)
    {
    iterator.Set(255);
    }

  TImageType::Pointer originalImage = TImageType::New();
  CreateImage(originalImage);

  QuickView viewer;
  viewer.AddImage(originalImage.GetPointer());
  viewer.AddImage(image.GetPointer());
  viewer.Visualize();

  return EXIT_SUCCESS;
}

void CreateImage(TImageType::Pointer image)
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

      image->SetPixel(pixelIndex, 100);
      }
    }
}