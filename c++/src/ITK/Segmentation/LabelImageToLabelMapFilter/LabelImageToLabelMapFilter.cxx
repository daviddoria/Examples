#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkLabelImageToLabelMapFilter.h"
#include "itkConnectedComponentImageFilter.h"

typedef itk::Image<unsigned char, 2>  ImageType;

void CreateImage(ImageType::Pointer image);

int main(int, char *[])
{
  ImageType::Pointer image = ImageType::New();
  CreateImage(image);

  typedef itk::ConnectedComponentImageFilter <ImageType, ImageType >
    ConnectedComponentImageFilterType;
   ConnectedComponentImageFilterType::Pointer connectedComponentImageFilter
    = ConnectedComponentImageFilterType::New ();
  connectedComponentImageFilter->SetInput(image);
  connectedComponentImageFilter->Update();

  typedef itk::LabelImageToLabelMapFilter <ImageType>
    LabelImageToLabelMapFilterType;
   LabelImageToLabelMapFilterType::Pointer labelImageToLabelMapFilter
    = LabelImageToLabelMapFilterType::New ();
  labelImageToLabelMapFilter->SetInput(connectedComponentImageFilter->GetOutput());
  labelImageToLabelMapFilter->Update();

  // The output of this filter is an itk::LabelMap, which contains itk::LabelObject's
  std::cout << "There are " << labelImageToLabelMapFilter->GetOutput()->GetNumberOfLabelObjects() << " objects." << std::endl;
 
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

  // Make a square
  for(unsigned int r = 20; r < 80; r++)
    {
    for(unsigned int c = 30; c < 100; c++)
      {
      ImageType::IndexType pixelIndex;
      pixelIndex[0] = r;
      pixelIndex[1] = c;

      image->SetPixel(pixelIndex, 255);
      }
  }

  // Make another square
  for(unsigned int r = 100; r < 130; r++)
    {
    for(unsigned int c = 115; c < 160; c++)
      {
      ImageType::IndexType pixelIndex;
      pixelIndex[0] = r;
      pixelIndex[1] = c;

      image->SetPixel(pixelIndex, 255);
      }
  }
}
