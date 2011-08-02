#include "itkImage.h"
#include "itkMirrorPadImageFilter.h"
#include "itkChangeInformationImageFilter.h"

typedef itk::Image<unsigned char, 2>  ImageType;

static void CreateImage(ImageType::Pointer image);

int main(int argc, char *argv[])
{
  ImageType::Pointer image = FloatImageType::New();
  CreateImage(image);

  typedef itk::MirrorPadImageFilter <TImage, TImage> MirrorPadImageFilterType;
  itk::Size<2> extendSize;
  extendSize.Fill(10);
 
  typename MirrorPadImageFilterType::Pointer padFilter = MirrorPadImageFilterType::New();
  padFilter->SetInput(image);
  padFilter->SetPadBound(extendSize);
  padFilter->Update();
    
//   typedef  itk::ChangeInformationImageFilter< UnsignedCharImageType  > WriterType;
//   WriterType::Pointer writer = WriterType::New();
//   writer->SetFileName("output.png");
//   writer->SetInput(zeroCrossingImageFilter->GetOutput());
//   writer->Update();

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
  
  // Make half of the image negative
  for(unsigned int i = 0; i < 100; ++i)
    {
    for(unsigned int j = 0; j < 50; ++j)
      {
      itk::Index<2> index;
      index[0] = i;
      index[1] = j;
      image->SetPixel(index, 1);
      }
    }
}
