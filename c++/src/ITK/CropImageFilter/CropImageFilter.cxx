#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkCropImageFilter.h"

typedef itk::Image<unsigned char, 2>  ImageType;

void CreateImage(ImageType::Pointer image);

int main(int, char *[])
{
  ImageType::Pointer image = ImageType::New();
  CreateImage(image);

  typedef itk::CropImageFilter <ImageType, ImageType>
    CropImageFilterType;

  CropImageFilterType::Pointer cropFilter
    = CropImageFilterType::New();
  cropFilter->SetInput(image);
  // The SetBoundaryCropSize( cropSize ) method specifies the size of the boundary to
  // be cropped at both the uppper & lower ends of the image
  // eg. cropSize/2 pixels will be removed at both upper & lower extents
  ImageType::SizeType cropSize = {{10,15}};
  cropFilter->SetBoundaryCropSize(cropSize);
  // Doxygen shows these as protected attributes. Should they be available to a user
  // The Test for CropImageFilter shows the below usage. Is it correct ?
  cropFilter->SetUpperBoundaryCropSize(cropSize);
  //cropFilter->SetLowerBoundaryCropSize(cropSize);
  cropFilter->Update();

  return EXIT_SUCCESS;
}

void CreateImage(ImageType::Pointer image)
{
  // Create an image with 2 connected components
  ImageType::RegionType region;
  ImageType::IndexType start;
  start[0] = 0;
  start[1] = 0;

  ImageType::SizeType size;
  unsigned int NumRows = 200;
  unsigned int NumCols = 300;
  size[0] = NumRows;
  size[1] = NumCols;

  region.SetSize(size);
  region.SetIndex(start);

  image->SetRegions(region);
  image->Allocate();
  image->FillBuffer( 0 );

  // Make a rectangle, centered at (100,150) with sides 160 & 240
  // This provides a 20 x 30 border around the square for the crop filter to remove
  for(unsigned int r = 20; r < 180; r++)
  {
    for(unsigned int c = 30; c < 270; c++)
    {
      ImageType::IndexType pixelIndex;
      pixelIndex[0] = r;
      pixelIndex[1] = c;

      image->SetPixel(pixelIndex, 200);
    }
  }
}
