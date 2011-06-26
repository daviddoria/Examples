#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkJoinImageFilter.h"
#include "itkVectorImageToImageAdaptor.h"

typedef itk::Image<unsigned char, 2>  ImageType;

static void CreateImage(ImageType::Pointer image, unsigned char value);

int main(int, char *[])
{
  ImageType::Pointer image1 = ImageType::New();
  CreateImage(image1, 0);

  ImageType::Pointer image2 = ImageType::New();
  CreateImage(image2, 10);

  typedef itk::JoinImageFilter<ImageType, ImageType> JoinImageFilterType;

  JoinImageFilterType::Pointer joinFilter = JoinImageFilterType::New();
  joinFilter->SetInput1(image1);
  joinFilter->SetInput2(image2);
  joinFilter->Update();

  itk::Index<2> index;
  index[0] = 0;
  index[1] = 0;

  std::cout << static_cast<int>(joinFilter->GetOutput()->GetPixel(index)[0]) << std::endl;
  std::cout << static_cast<int>(joinFilter->GetOutput()->GetPixel(index)[1]) << std::endl;

  return EXIT_SUCCESS;
}

void CreateImage(ImageType::Pointer image, unsigned char value)
{
  // Create an image
  ImageType::IndexType start;
  start.Fill(0);

  ImageType::SizeType size;
  size.Fill(100);

  ImageType::RegionType region(start,size);

  image->SetRegions(region);
  image->Allocate();
  image->FillBuffer(value);

}
