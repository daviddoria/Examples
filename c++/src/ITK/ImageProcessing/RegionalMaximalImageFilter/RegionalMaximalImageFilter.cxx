#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkRegionalMaximaImageFilter.h"
 
typedef itk::Image<unsigned char, 2>  ImageType;
 
static void CreateImage(ImageType::Pointer image);
 
int main(int, char *[])
{
  ImageType::Pointer image = ImageType::New();
  CreateImage(image);
 
  typedef itk::RegionalMaximaImageFilter <ImageType, ImageType >
    RegionalMaximaImageFilter;
 
  RegionalMaximaImageFilter::Pointer filter
    = RegionalMaximaImageFilter::New ();
  filter->SetInput(image);
 
  typedef itk::ImageFileWriter< ImageType > WriterType;
  WriterType::Pointer writer = WriterType::New();
 
  writer->SetFileName("input.png");
  writer->SetInput( image );
  writer->Update();
 
  writer->SetFileName("output.png");
  writer->SetInput( filter->GetOutput() );
  writer->Update();
 
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
  size[0] = NumCols;
  size[1] = NumRows;
 
  region.SetSize(size);
  region.SetIndex(start);
 
  image->SetRegions(region);
  image->Allocate();
 
  // Make two intensity blobs
  for(unsigned int r = 0; r < NumRows; r++)
    {
    for(unsigned int c = 0; c < NumCols; c++)
      {
      ImageType::IndexType pixelIndex;
      pixelIndex[0] = c;
      pixelIndex[1] = r;
 
      double c1 = c - 100.0;
      double c2 = c - 200.0;
 
      double rr = r - 100.0;
 
      // purposely use 270,257 since it is > 255
      double v1 = 270.0 - vcl_sqrt( rr*rr + c1*c1 );
      double v2 = 257.0 - vcl_sqrt( rr*rr + c2*c2 );
 
      double maxv = v1;
      if( maxv < v2 )  maxv = v2;
 
      double val = maxv;
 
      if( val <   0.0 ) val = 0.0;
      if( val > 255.0 ) val = 255.0;
 
      image->SetPixel(pixelIndex, static_cast<unsigned char>(val) );
      }
    }
}
