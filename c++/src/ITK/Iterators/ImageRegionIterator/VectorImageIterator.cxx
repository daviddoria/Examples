#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageRegionIterator.h"
#include "itkCovariantVector.h"

int main(int, char*[])
{
  typedef itk::Image<itk::CovariantVector<double, 3>, 2>  ImageType;

  ImageType::Pointer image = ImageType::New();

  ImageType::SizeType size;
  size[0] = 2;
  size[1] = 2;

  ImageType::IndexType index;
  index[0] = 0;
  index[1] = 0;

  ImageType::RegionType region;
  region.SetSize(size);
  region.SetIndex(index);

  image->SetRegions(region);
  image->Allocate();

  itk::ImageRegionIterator<ImageType> imageIterator(image,image->GetLargestPossibleRegion());

  imageIterator.GoToBegin();
  while(!imageIterator.IsAtEnd())
    {
    itk::CovariantVector<double, 3> pixel = imageIterator.Get();
    ++imageIterator;
    }

  return EXIT_SUCCESS;
}
